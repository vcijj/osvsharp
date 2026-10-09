/* osvsharp.c — DJI Osmo 360 .OSV → sharp frames, one command, pure C (Win32).
 *
 * Pipeline (no intermediate frame files; decode is delegated to an ffmpeg
 * subprocess, frames travel over a pipe):
 *
 *   probe   ffmpeg -hwaccel <X> ... -frames:v 1 -f null -
 *           picks X from cuda → vulkan → d3d11va; falls back to software.
 *   pass A  ffmpeg [-hwaccel X] -i IN -map 0:v:T -vf scale=S:S,format=gray
 *                   -f rawvideo pipe:1
 *           → score every frame: variance of the Laplacian on an S×S luma
 *             thumbnail (border pixels excluded), then spirula-studio's
 *             sliding-window selection: every `skip` source frames write the
 *             sharpest of the last `keep` candidate frames.
 *   pass B  ffmpeg [-hwaccel X] -i IN -map 0:v:T -vf "select='eq(n,i)+...'"
 *                   -c:v mjpeg -q:v Q -f mjpeg pipe:1
 *           → split the JPEG stream, write one file per winner named by its
 *             source frame index.
 *
 *           With -mask N (default 80), pass B adds a full-resolution
 *           circular mask PNG + maskedmerge to the filter graph, so
 *           everything outside the lens circle is blacked out during the
 *           one and only mjpeg encode: no extra decode/encode pass, no
 *           generation loss.
 *
 * The metric and the window arithmetic are ports of spirula-studio's
 * src/video/shaders/video.slang and src/app/FrameExtract.cpp, so results are
 * comparable with that tool's `spirula sam extract`.
 *
 * Build:  clang -std=c99 -O2 osvsharp.c -o osvsharp.exe
 * Usage:  osvsharp <input.OSV> [outdir] [-s skip] [-k keep] [-m min_score]
 *                     [-t track] [-q jpeg_q] [-b thumb] [-mask pct]
 *                     [--ffmpeg PATH] [--hwaccel NAME] [--no-hwaccel]
 */

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>
#include <shellapi.h>
#include <shlobj.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <wchar.h>

/* 圆形遮罩 PNG 生成（边缘涂黑用）：内嵌公有领域单头写库，编译进 exe，
 * 运行时仍是无依赖单文件。涂黑本体已并入抽帧编码滤镜（见 make_mask_png
 * 与 pass_b），不再引用图像解码库。 */
#define STBIW_WINDOWS_UTF8
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "comctl32.lib")

#define DEF_SKIP    8      /* 25 fps source → ~3 written frames per second */
#define DEF_JPEG_Q  2      /* ffmpeg mjpeg -q:v (lower = better)            */
#define DEF_THUMB   512    /* spirula's thumbnail size                      */
#define DEF_MASK    95     /* 涂黑保留半径百分数（0=关），抽帧编码时同步执行  */
/* 动态间隔模式（--diff > 0 启用）的出图间隔伸缩范围（源帧数） */
#define DEF_ADAPT_MIN_GAP 4    /* 最密 ≈6 张/秒 @25fps（快移段防帧数爆炸） */
#define DEF_ADAPT_MAX_GAP 32   /* 最稀兜底：静止段最多隔 32 帧出一张      */

/* --------------------------------------------------------- 参数详解（速查）
 *
 * 界面输入框（括号内为等价命令行参数）的含义与设置方法。
 * 两种选帧模式：
 *   固定间隔（默认）：源视频按“抽帧间隔 skip”分组，每组末尾 keep
 *     帧评清晰度分，最清晰的一张出图；分数低于“最低清晰度”的整组放弃。
 *   动态间隔（“变化阈值”>0 启用）：每帧与上一张出图帧算平均灰度差，
 *     变化攒够阈值才结算出图（窗内仍挑最清晰，仍受最低清晰度过滤）；
 *     变化快→出图密、静止→自动变稀，间隔在 min_gap~max_gap 帧间伸缩。
 * 双目文件两轨共用轨0 上测得的布点：cam0/cam1 输出帧号严格一致、
 * 逐号配对；轨1 跳过评分，也不生成 cam1 的 csv。
 *
 * ┌────────────┬──────┬────────────┬────────────────────────────────┐
 * │ 界面标签    │ 参数  │ 默认        │ 作用                           │
 * ├────────────┼──────┼────────────┼────────────────────────────────┤
 * │ 抽帧间隔    │ skip │ 8          │ 每 skip 帧输出 1 张             │
 * │ 锐度窗口    │ keep │ 留空=skip/2 │ 每组末尾 keep 帧里挑最清晰      │
 * │ 变化阈值    │ diff │ 0（固定）   │ >0 启用动态间隔，见下           │
 * │ 最低清晰度  │ min  │ 0（不过滤） │ 分数低于此值的胜者不出图        │
 * │ JPEG质量    │ q    │ 2          │ mjpeg -q:v，越小越清晰          │
 * │ 评分缩略图  │thumb │ 512        │ 打分用的灰度图边长，影响分数刻度│
 * │ 涂黑半径%   │ mask │ 95（0=关） │ 抽帧时圆外涂黑（仅双目生效）    │
 * └────────────┴──────┴────────────┴────────────────────────────────┘
 *
 * 【抽帧间隔 skip（-s，默认 8）】
 *   决定出图密度：输出节奏 ≈ 帧率 ÷ skip。25fps、skip=8 → 约 3 张/秒；
 *   skip=4 约 6 张/秒。想要更密调小、更稀调大，对清晰度本身无影响。
 *   动态间隔模式下此参数不参与。
 *
 * 【锐度窗口 keep（-k，界面留空 = 自动取 skip/2 四舍五入）】
 *   每组 skip 帧里只有“末尾 keep 帧”参与评分，从中挑分数最高的一张
 *   输出。例：skip=8、keep=4 → 每组的第 4~7 帧打分，最清晰者出图，
 *   前 4 帧只用来垫时间轴。
 *     keep 越大  → 候选越多，出图通常越清晰；但出图时刻离组尾越远，
 *                  时间戳抖动最大可达 (keep-1)/帧率 秒。
 *     keep = skip → 全组参评，清晰度收益最大，抖动也最大。
 *     keep = 1    → 只看组内最后一帧，等于不挑（仅剩 min 过滤）。
 *     填 0        → 关闭挑选：固定取每组第 1 帧（0、skip、2·skip…），
 *                  仍受“最低清晰度”过滤。想要确定性时间戳时用。
 *   建议范围 1~skip；超过 skip 时窗口跨组滑动，一般用不到。
 *   动态间隔模式下此参数不参与。
 *
 * 【变化阈值 diff（--diff，默认 0 = 固定间隔模式）】
 *   填大于 0 的数即切换为“动态间隔”：每个候选帧与上一张出图帧的
 *   缩略图算平均灰度差（MAD，0~255 刻度，记进 csv 的 diff 列），
 *   差值攒够阈值才关窗出图——窗内仍挑最清晰一张，仍受“最低清晰度”
 *   过滤。变化快→出图密、静止→自动变稀，相邻出图帧的画面变化量
 *   大致恒定，给 SfM 喂图最合适。
 *   出图间隔被钳在 min_gap~max_gap 帧之间（--min-gap/--max-gap，
 *   默认 4~32；25fps 下 ≈ 每秒 0.8~6 张），急速运动不会帧数爆炸，
 *   长静止段也有兜底出图。此模式下 skip/keep 均被忽略。
 *   ⚠ 阈值刻度与场景内容、缩略图尺寸(-b)相关：先用固定模式跑一遍，
 *     看 csv 里 diff 列在静止段/走动段的分布，把阈值定在两者之间；
 *     调大 → 出图更稀，调小 → 更密。实测 8K 行走素材：8 ≈ 默认密度
 *     （约 3 张/秒），12 约减半，常用 5~12。
 *
 * 【最低清晰度 min_score（-m，默认 0 = 全部保留）】
 *   分数定义：先把帧缩成 thumb×thumb 的 8 位灰度图，算拉普拉斯响应
 *   的方差（见 laplacian_var()）。边缘/纹理多 → 方差大 = 清晰；运动
 *   模糊、失焦、大面积天空/墙面 → 方差小 = 模糊。
 *   每组的胜者分数 < min_score 时该组整组放弃出图，用来丢掉模糊照片。
 *   ⚠ 数值没有绝对标准：与缩略图尺寸(-b)、场景内容强相关，只能在
 *     同一批参数下相对比较；改了 -b 阈值要重新标定。
 *   标定方法：先用默认 0 跑一遍 → 打开输出目录旁的 sharpness_camN.csv
 *   （每行：frame,score,chosen）看分数分布，或看日志里打印的
 *   "score min=xx avg=xx max=xx"，把阈值定在模糊段与清晰段之间，
 *   通常先试 min~avg 之间的某个值，跑完看丢的比例再微调。
 *
 * 【JPEG质量 q（-q，默认 2）】ffmpeg mjpeg 的 -q:v：2 接近视觉无损，
 *   想减小体积可到 4~5，再大画质明显下降。范围 1~31。
 * 【评分缩略图 thumb（-b，默认 512）】只影响打分速度与分数刻度，不
 *   影响输出照片的分辨率（输出永远是原始分辨率重新解码的 JPEG）。
 *
 * 【GPS 遥测（自动，无需参数）】视频旁有同名 .SRT（大疆导出的飞行
 *   遥测字幕，每帧一条 [latitude/longitude/rel_alt/abs_alt]）时，每张
 *   出图在写盘时自动注入 EXIF GPS（经纬度+海拔+UserComment 摘要），
 *   Metashape/Pix4D 等建模软件可直接读取做地理约束；无 SRT 行为不变。
 *   帧号→时间用探测到的帧率对齐；本代 SRT 不含姿态角，故不写姿态。
 */

/* ------------------------------------------------- progress/cancel hooks */

/* The GUI installs these; the CLI leaves g_hk NULL and LOG falls back to
 * printf.  file_status phases: 0=probing 1=scoring 2=writing 3=track done. */
typedef struct {
    void (*log)(void *user, const char *utf8_line);
    void (*file_status)(void *user, int file_idx, int phase, int track, long long a);
    void (*file_done)(void *user, int file_idx, int rc);
    void *user;
    volatile int *cancel;
} Hooks;

static Hooks *g_hk;
static volatile int g_cancel;
static HANDLE g_cur_child;   /* current ffmpeg, terminated on cancel */

static void LOG(const char *fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (g_hk && g_hk->log) g_hk->log(g_hk->user, buf);
    else { printf("%s", buf); fflush(stdout); }
}

static void STATUS(int file_idx, int phase, int track, long long a) {
    if (g_hk && g_hk->file_status) g_hk->file_status(g_hk->user, file_idx, phase, track, a);
}

static int cancelled(void) { return g_hk && g_hk->cancel && *g_hk->cancel; }

static int g_file_idx = -1;   /* STATUS() target, set by process_input */

static void take_child(HANDLE h) {
    InterlockedExchangePointer((void *volatile *)&g_cur_child, h);
}
static void drop_child(void) {
    InterlockedExchangePointer((void *volatile *)&g_cur_child, NULL);
}

/* ------------------------------------------------------------------ utils */

static wchar_t *utf8_to_w(const char *s) {
    int n = MultiByteToWideChar(CP_UTF8, 0, s, -1, NULL, 0);
    wchar_t *w = malloc((size_t)n * sizeof(wchar_t));
    MultiByteToWideChar(CP_UTF8, 0, s, -1, w, n);
    return w;
}

static char *w_to_utf8(const wchar_t *ws) {
    int n = WideCharToMultiByte(CP_UTF8, 0, ws, -1, NULL, 0, NULL, NULL);
    char *s = malloc((size_t)n);
    WideCharToMultiByte(CP_UTF8, 0, ws, -1, s, n, NULL, NULL);
    return s;
}

static void mkdirs_w(const wchar_t *path) {
    wchar_t *tmp = _wcsdup(path);
    for (wchar_t *p = tmp; *p; p++) {
        if (*p == L'\\' || *p == L'/') {
            if (p == tmp + 1 && tmp[1] == L':' && (tmp[2] == L'\\' || tmp[2] == L'/'))
                continue; /* keep the drive root */
            wchar_t save = *p;
            *p = 0;
            if (wcslen(tmp) > 0) CreateDirectoryW(tmp, NULL);
            *p = save;
        }
    }
    if (wcslen(tmp) > 0) CreateDirectoryW(tmp, NULL);
    free(tmp);
}

/* ------------------------------------------------------- command line build */

typedef struct { wchar_t *s; size_t len, cap; } Cmd;

static void cmd_reserve(Cmd *c, size_t extra) {
    if (c->len + extra + 1 <= c->cap) return;
    while (c->cap < c->len + extra + 1) c->cap = c->cap ? c->cap * 2 : 256;
    c->s = realloc(c->s, c->cap * sizeof(wchar_t));
}

static void cmd_raw(Cmd *c, const wchar_t *arg) { /* append raw text */
    size_t n = wcslen(arg);
    cmd_reserve(c, n);
    memcpy(c->s + c->len, arg, n * sizeof(wchar_t));
    c->len += n;
    c->s[c->len] = 0;
}

static void cmd_add(Cmd *c, const wchar_t *arg) { /* append one argv word */
    cmd_reserve(c, wcslen(arg) + 3);
    cmd_raw(c, c->len ? L" " : L"");
    int quote = wcspbrk(arg, L" \t\"") != NULL;
    if (!quote) { cmd_raw(c, arg); return; }
    cmd_raw(c, L"\"");
    for (const wchar_t *p = arg; *p; p++) {
        if (*p == L'"' || *p == L'\\') cmd_raw(c, L"\\");
        wchar_t one[2] = { *p, 0 };
        cmd_raw(c, one);
    }
    cmd_raw(c, L"\"");
}

/* ffmpeg wants e.g. "-hwaccel" and "cuda" as two separate argv words; never
 * build an option string that contains a space. */
static void cmd_add2(Cmd *c, const wchar_t *flag, const wchar_t *value) {
    cmd_add(c, flag);
    cmd_add(c, value);
}

static void cmd_addf(Cmd *c, const wchar_t *fmt, ...) {
    wchar_t buf[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnwprintf(buf, 512, fmt, ap);
    va_end(ap);
    cmd_add(c, buf);
}

/* ------------------------------------------------------------ child pipes */

typedef struct { HANDLE r; PROCESS_INFORMATION pi; } Child;

static int spawn(const wchar_t *cmdline, Child *c, int stderr_mode) {
    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, TRUE };
    HANDLE rh, wh;
    if (!CreatePipe(&rh, &wh, &sa, 0)) return -1;
    SetHandleInformation(wh, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT);

    STARTUPINFOW si;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = wh;
    si.hStdError = stderr_mode == 2 ? wh
                 : stderr_mode == 1 ? NULL
                 : GetStdHandle(STD_ERROR_HANDLE);
    si.hStdInput = NULL; /* every ffmpeg call passes -nostdin */

    wchar_t *cl = _wcsdup(cmdline);
    BOOL ok = CreateProcessW(NULL, cl, NULL, NULL, TRUE,
                             CREATE_NO_WINDOW, NULL, NULL, &si, &c->pi);
    free(cl);
    CloseHandle(wh);
    if (!ok) { CloseHandle(rh); return -1; }
    c->r = rh;
    return 0;
}

/* 1 = got all n bytes, 0 = eof or error (partial data lost on eof) */
static int read_exact(HANDLE h, void *buf, size_t n) {
    uint8_t *p = buf;
    while (n > 0) {
        DWORD got = 0;
        if (!ReadFile(h, p, (DWORD)(n > (1u << 30) ? (1u << 30) : n), &got, NULL))
            return 0;
        if (got == 0) return 0;
        p += got;
        n -= got;
    }
    return 1;
}

static int child_wait(Child *c) { /* exit code, -1 on failure */
    drop_child();
    CloseHandle(c->r);
    if (WaitForSingleObject(c->pi.hProcess, INFINITE) != WAIT_OBJECT_0) return -1;
    DWORD code = (DWORD)-1;
    GetExitCodeProcess(c->pi.hProcess, &code);
    CloseHandle(c->pi.hProcess);
    CloseHandle(c->pi.hThread);
    return (int)code;
}

/* run to completion ignoring stdout/stderr (probing) */
static int run_quiet(const wchar_t *cmdline) {
    Child c;
    if (spawn(cmdline, &c, 1) != 0) return -1;
    /* drain stdout so a chatty child cannot block */
    uint8_t sink[65536];
    for (;;) {
        DWORD got = 0;
        if (!ReadFile(c.r, sink, sizeof(sink), &got, NULL) || got == 0) break;
    }
    return child_wait(&c);
}

/* ------------------------------------------------------------- sharpness */

/* Variance of the 4-neighbour Laplacian on an S×S 8-bit luma thumbnail.
 * Border pixels excluded, matching spirula's laplacian_variance shader. */
static double laplacian_var(const uint8_t *g, int S) {
    double sum = 0.0, sqr = 0.0;
    long long n = 0;
    for (int y = 1; y < S - 1; y++) {
        const uint8_t *row = g + (size_t)y * S;
        for (int x = 1; x < S - 1; x++) {
            double lap = (double)row[x - 1] + row[x + 1] +
                         row[x - S] + row[x + S] - 4.0 * row[x];
            sum += lap;
            sqr += lap * lap;
            n++;
        }
    }
    if (!n) return 0.0;
    double mean = sum / (double)n;
    return sqr / (double)n - mean * mean;
}

/* Mean absolute difference of two 8-bit luma thumbnails (0~255 scale):
 * 动态间隔模式的帧间变化量。只在候选帧上算（缩略图已由 scale 滤镜
 * 重度平滑过，噪声残留远小于真实运动）。 */
static double frame_diff_mad(const uint8_t *a, const uint8_t *b, size_t n) {
    unsigned long long sum = 0;
    for (size_t k = 0; k < n; k++)
        sum += a[k] > b[k] ? (unsigned)(a[k] - b[k]) : (unsigned)(b[k] - a[k]);
    return n ? (double)sum / (double)n : 0.0;
}

/* ------------------------------------------- sliding-window frame selection */

typedef struct { long long idx; double score; double diff; } Cand;

typedef struct {
    int skip, keep;
    double min_score;
    /* 动态间隔模式（diff_th > 0 启用）：帧间差攒够阈值才关窗出图，
     * 间隔在 min_gap~max_gap 源帧之间伸缩；skip/keep 被忽略。 */
    int adaptive;
    double diff_th;
    int min_gap, max_gap, thumb;
    uint8_t *last_out;                  /* 上一张出图帧的缩略图（差值基准）*/
    uint8_t *best_thumb;                /* 当前窗内最清晰候选的缩略图      */
    long long last_out_idx;             /* -1 = 还没有基准帧               */
    long long win_start;                /* 当前窗首个候选的帧号            */
    Cand *win;  size_t nwin,  capwin;   /* chosen frames                */
    Cand *all;  size_t nall,  capall;   /* every candidate, for the csv */
    char  *chosen;                      /* parallel to all              */
    Cand *w; int wsize, wcap;           /* live window                  */
    long long fc;
} Select;

static void sel_init(Select *s, int skip, int keep, double min_score,
                     double diff_th, int min_gap, int max_gap, int thumb) {
    memset(s, 0, sizeof(*s));
    s->skip = skip;
    s->keep = keep;
    s->min_score = min_score;
    s->diff_th = diff_th;
    s->adaptive = diff_th > 0.0;
    s->min_gap = min_gap;
    s->max_gap = max_gap;
    s->thumb = thumb;
    s->last_out_idx = -1;
    /* 动态模式窗长上限 = max_gap；固定模式沿用 keep（keep==0 = 不挑）*/
    s->wcap = s->adaptive ? max_gap : (keep > 0 ? keep : 1);
    s->w = malloc(sizeof(Cand) * (size_t)s->wcap);
    size_t fsz = (size_t)thumb * thumb;
    s->last_out = malloc(fsz);
    s->best_thumb = malloc(fsz);
}

static void sel_free(Select *s) {
    free(s->win); free(s->all); free(s->chosen); free(s->w);
    free(s->last_out); free(s->best_thumb);
}

static void push_cand(Cand **v, size_t *n, size_t *cap, Cand c) {
    if (*n == *cap) {
        *cap = *cap ? *cap * 2 : 1024;
        *v = realloc(*v, sizeof(Cand) * *cap);
    }
    (*v)[(*n)++] = c;
}

/* 关窗结算：窗口里分数最高的候选标记为胜者，分数 >= min_score 才进
 * 最终输出列表；胜者缩略图成为下一次帧间差的基准。 */
static void sel_close_window(Select *s) {
    int best = 0;
    for (int k = 1; k < s->wsize; k++)
        if (s->w[k].score > s->w[best].score) best = k;
    /* mark chosen in `all` */
    for (size_t k = 0; k < s->nall; k++)
        if (s->all[k].idx == s->w[best].idx) { s->chosen[k] = 1; break; }
    if (s->w[best].score >= s->min_score)
        push_cand(&s->win, &s->nwin, &s->capwin, s->w[best]);
    memcpy(s->last_out, s->best_thumb, (size_t)s->thumb * s->thumb);
    s->last_out_idx = s->w[best].idx;
    s->wsize = 0;
}

/* Port of FrameExtract.cpp's decode loop: candidate test, window push,
 * and "write the sharpest of the window every `skip` source frames".
 *
 * 候选判定（keep>0）：((i+keep)%skip) < keep —— 把源帧按 skip 个一组
 * 切开后，每组只有末尾 keep 帧是候选（例 skip=8、keep=4 → 候选是组内
 * 第 4~7 帧）。keep==0 则退化为每组第 1 帧（i%skip==0）且不挑帧。
 * 每攒满一组（fc 到达 skip 的倍数）就把窗口里分数最高的候选标记为
 * 胜者；胜者分数 >= min_score 才进入最终输出列表。
 *
 * 动态间隔模式（sel_init 传 diff_th > 0）：候选 = 距上一张出图帧
 * >= min_gap 的所有帧；帧间差（与上一张出图帧的 MAD）>= diff_th 或
 * 窗长到达 max_gap 时关窗。帧 0 播种基准但不保证出图。分数在候选
 * 判定之后才算（非候选帧的 laplacian 是白算的，两种模式都省掉）。 */
static void sel_frame(Select *s, long long i, const uint8_t *thumb) {
    size_t fsz = (size_t)s->thumb * s->thumb;

    if (s->adaptive) {
        if (s->last_out_idx < 0) {            /* 第一帧播种差值基准 */
            memcpy(s->last_out, thumb, fsz);
            s->last_out_idx = i;
        }
        if (i < s->last_out_idx + s->min_gap) return;  /* 离上张太近 */
        double score = laplacian_var(thumb, s->thumb);
        double d = frame_diff_mad(thumb, s->last_out, fsz);
        if (s->wsize == 0) s->win_start = i;
        push_cand(&s->all, &s->nall, &s->capall, (Cand){ i, score, d });
        s->chosen = realloc(s->chosen, s->nall);
        s->chosen[s->nall - 1] = 0;
        s->w[s->wsize].idx = i;
        s->w[s->wsize].score = score;
        s->wsize++;
        int is_best = 1;
        for (int k = 0; k < s->wsize - 1; k++)
            if (s->w[k].score >= score) { is_best = 0; break; }
        if (is_best) memcpy(s->best_thumb, thumb, fsz);
        if (d >= s->diff_th || i - s->win_start + 1 >= s->max_gap)
            sel_close_window(s);
        return;
    }

    int cand = s->keep > 0
        ? (int)(((i + s->keep) % s->skip) < s->keep)
        : (int)((i % s->skip) == 0);
    if (!cand) { s->fc++; return; }

    double score = laplacian_var(thumb, s->thumb);
    if (s->last_out_idx < 0) {                /* 第一个候选播种差值基准 */
        memcpy(s->last_out, thumb, fsz);
        s->last_out_idx = i;
    }
    double d = frame_diff_mad(thumb, s->last_out, fsz);
    push_cand(&s->all, &s->nall, &s->capall, (Cand){ i, score, d });
    s->chosen = realloc(s->chosen, s->nall);
    s->chosen[s->nall - 1] = 0;

    if (s->wsize == s->wcap) {
        /* 防御路径，正常不会到（每组候选数不会超过 wcap，见上注释）；
         * 挤掉最旧候选后 best_thumb 可能指向已出窗的帧，但此路不触发 */
        memmove(s->w, s->w + 1, sizeof(Cand) * (size_t)(s->wcap - 1));
        s->wsize--;
    }
    s->w[s->wsize].idx = i;
    s->w[s->wsize].score = score;
    s->wsize++;
    int is_best = 1;
    for (int k = 0; k < s->wsize - 1; k++)
        if (s->w[k].score >= score) { is_best = 0; break; }
    if (is_best) memcpy(s->best_thumb, thumb, fsz);

    if (s->keep > 0) s->fc++;
    int write_now = (s->fc % s->skip) == 0;
    if (s->keep == 0) s->fc++;
    if (!write_now) return;

    sel_close_window(s);
}

/* pass_a 结束时冲掉动态模式最后一个不完整窗口。固定模式保持老行为
 * （尾部不满一组的候选不结算），保证输出数量与历史版本零漂移。 */
static void sel_flush(Select *s) {
    if (s->adaptive && s->wsize > 0) sel_close_window(s);
}

/* ------------------------------------------------------------ pass runners */

typedef struct {
    const wchar_t *ffmpeg;   /* ffmpeg executable                */
    const wchar_t *input;    /* the .OSV path                    */
    const wchar_t *hw;       /* L"" = software                   */
    int thumb, jpeg_q;
    int vw, vh;              /* current track resolution (mask)  */
} Ctx;

/* returns malloc'd probed hwaccel name, L"" for software, or NULL when even
 * software cannot decode one frame of this track (no such track / bad file) */
static wchar_t *probe_hwaccel(const Ctx *cx, int track, int quiet_ok) {
    static const wchar_t *order[] = { L"cuda", L"vulkan", L"d3d11va", NULL };
    for (int i = 0; order[i]; i++) {
        Cmd c = {0};
        cmd_add(&c, cx->ffmpeg);
        cmd_add(&c, L"-v");
        cmd_add(&c, L"error");
        cmd_add(&c, L"-nostdin");
        cmd_add2(&c, L"-hwaccel", order[i]);
        cmd_add(&c, L"-i");
        cmd_add(&c, cx->input);
        wchar_t mapspec[16]; swprintf(mapspec, 16, L"0:v:%d", track);
        cmd_add2(&c, L"-map", mapspec);
        cmd_add(&c, L"-frames:v");
        cmd_add(&c, L"1");
        cmd_add(&c, L"-fps_mode");
        cmd_add(&c, L"passthrough");
        cmd_add(&c, L"-f");
        cmd_add(&c, L"null");
        cmd_add(&c, L"-");
        int rc = run_quiet(c.s);
        free(c.s);
        if (rc == 0) {
            if (!quiet_ok) wprintf(L"hwaccel: %ls\n", order[i]);
            return _wcsdup(order[i]);
        }
    }
    /* software fallback must also be verified: it is what proves the track
     * exists at all */
    {
        Cmd c = {0};
        cmd_add(&c, cx->ffmpeg);
        cmd_add(&c, L"-v");
        cmd_add(&c, L"error");
        cmd_add(&c, L"-nostdin");
        cmd_add(&c, L"-i");
        cmd_add(&c, cx->input);
        wchar_t mapspec[16]; swprintf(mapspec, 16, L"0:v:%d", track);
        cmd_add2(&c, L"-map", mapspec);
        cmd_add(&c, L"-frames:v");
        cmd_add(&c, L"1");
        cmd_add(&c, L"-fps_mode");
        cmd_add(&c, L"passthrough");
        cmd_add(&c, L"-f");
        cmd_add(&c, L"null");
        cmd_add(&c, L"-");
        int rc = run_quiet(c.s);
        free(c.s);
        if (rc != 0) return NULL;
    }
    if (!quiet_ok) wprintf(L"hwaccel: none (software decode)\n");
    return _wcsdup(L"");
}

/* pass A: score every frame; fills `sel`. Returns decoded frame count or -1 */
static long long pass_a(const Ctx *cx, int track, Select *sel) {
    Cmd c = {0};
    cmd_add(&c, cx->ffmpeg);
    cmd_add(&c, L"-v");
    cmd_add(&c, L"error");
    cmd_add(&c, L"-nostdin");
    if (wcslen(cx->hw)) cmd_add2(&c, L"-hwaccel", cx->hw);
    cmd_add(&c, L"-i");
        cmd_add(&c, cx->input);
    wchar_t mapspec[16]; swprintf(mapspec, 16, L"0:v:%d", track);
        cmd_add2(&c, L"-map", mapspec);
    cmd_add(&c, L"-fps_mode");
    cmd_add(&c, L"passthrough");
    wchar_t vf[64]; swprintf(vf, 64, L"scale=%d:%d,format=gray", cx->thumb, cx->thumb);
    cmd_add2(&c, L"-vf", vf);
    cmd_add(&c, L"-f");
    cmd_add(&c, L"rawvideo");
    cmd_add(&c, L"pipe:1");

    Child ch;
    if (spawn(c.s, &ch, 0) != 0) { free(c.s); return -1; }
    free(c.s);
    take_child(ch.pi.hProcess);

    size_t fsz = (size_t)cx->thumb * cx->thumb;
    uint8_t *frame = malloc(fsz);
    long long i = 0;
    while (read_exact(ch.r, frame, fsz)) {
        if (cancelled()) break;
        sel_frame(sel, i, frame);
        if (++i % 250 == 0) STATUS(g_file_idx, 1, track, i);
    }
    sel_flush(sel);
    free(frame);
    int rc = child_wait(&ch);
    if (cancelled()) return -2;
    if (i == 0) {
        LOG("pass A: ffmpeg decoded nothing (exit %d)\n", rc);
        return -1;
    }
    if (rc != 0)
        LOG("pass A: ffmpeg exited %d after %lld frames\n", rc, i);
    return i;
}

/* ------------------------------------------------------------ GPS → EXIF */
/* 大疆把每帧 GPS/海拔遥测写在视频旁同名 .SRT 的字幕条目里（本机实测
 * 字段：[latitude: x] [longitude: x] [rel_alt: x abs_alt: x]，40ms/条
 * 与视频帧一一对应；这一代 SRT 不含姿态角，只有定位）。出图时按帧号
 * 查到对应时刻的遥测，在写盘阶段于 SOI 后注入 APP1-Exif 段（GPS IFD +
 * UserComment）——纯 C 手写，不重编码，像素零改动。无 SRT 行为不变。 */

typedef struct { double t0, lat, lon, rel_alt, abs_alt; } SrtEntry;
typedef struct { SrtEntry *e; size_t n; double fps; } Telem;

static void telem_free(Telem *tm) { free(tm->e); tm->e = NULL; tm->n = 0; }

static double srt_ts(const char *s) {   /* "00:01:02,345" -> 62.345 */
    int h = 0, m = 0, sec = 0, ms = 0;
    if (sscanf(s, "%d:%d:%d,%d", &h, &m, &sec, &ms) != 4) return -1.0;
    return h * 3600.0 + m * 60.0 + sec + ms / 1000.0;
}

/* 取 "[key: value]" 里 value 的 strtod 起点；找不到返回 NULL */
static const char *srt_field(const char *s, const char *key) {
    const char *p = strstr(s, key);
    if (!p) return NULL;
    p += strlen(key);
    while (*p == ' ') p++;
    return p;
}

/* 解析整份 SRT，返回条目数（0 = 文件在但没有可定位条目，-1 = 打不开）*/
static int srt_load(const wchar_t *path, Telem *tm) {
    FILE *f = _wfopen(path, L"rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > (1L << 30)) { fclose(f); return 0; }
    char *buf = malloc((size_t)sz + 1);
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = 0;
    size_t cap = 4096, n = 0;
    SrtEntry *e = malloc(cap * sizeof *e);
    SrtEntry cur = {0};
    int have_ts = 0, have_gps = 0;
    for (char *line = buf, *nl; line; line = nl ? nl + 1 : NULL) {
        nl = strchr(line, '\n');
        if (nl) *nl = 0;
        size_t L = strlen(line);
        if (L && line[L - 1] == '\r') line[L - 1] = 0;
        if (strstr(line, "-->")) {          /* 时间轴行 = 新条目开始 */
            if (have_ts && have_gps) {
                if (n == cap) { cap *= 2; e = realloc(e, cap * sizeof *e); }
                e[n++] = cur;
            }
            have_ts = have_gps = 0;
            cur = (SrtEntry){0};
            double t = srt_ts(line);
            if (t >= 0) { cur.t0 = t; have_ts = 1; }
            continue;
        }
        const char *v;
        /* key 不含左括号：兼容 "[ latitude:" / "(latitude:" 一类变体
         * （strtod 自跳空格与符号），同 DJI-SRT2GeoParser 的宽松正则 */
        if ((v = srt_field(line, "latitude:"))  != NULL) { cur.lat = strtod(v, NULL); have_gps = 1; }
        if ((v = srt_field(line, "longitude:")) != NULL) cur.lon = strtod(v, NULL);
        if ((v = srt_field(line, "rel_alt:"))   != NULL) cur.rel_alt = strtod(v, NULL);
        if ((v = srt_field(line, "abs_alt:"))   != NULL) cur.abs_alt = strtod(v, NULL);
    }
    if (have_ts && have_gps) {
        if (n == cap) { cap *= 2; e = realloc(e, cap * sizeof *e); }
        e[n++] = cur;
    }
    free(buf);
    tm->e = e;
    tm->n = n;
    return (int)n;
}

/* 帧号 -> 遥测条目。fps 已知按时间对齐（t = frame/fps，取 t0<=t 的最后
 * 一条）；fps 未知退化为按条目序号直对（DJI SRT 首条对应视频首帧），
 * 超出范围钳到最后一条（宁可有近似定位也不丢）。 */
static const SrtEntry *telem_lookup(const Telem *tm, long long frame) {
    if (!tm || tm->n == 0) return NULL;
    size_t lo = 0, hi = tm->n - 1;
    if (tm->fps > 0) {
        double t = (double)frame / tm->fps;
        /* SRT 时间戳与 t 各自有浮点舍入，边界帧（如 164*0.04 vs 164/25）
         * 可能差一个 ULP 翻转条目，给 1/4 帧容差 */
        double eps = 0.25 / tm->fps;
        if (t + eps < tm->e[0].t0) return &tm->e[0];
        while (lo < hi) {
            size_t mid = (lo + hi + 1) / 2;
            if (tm->e[mid].t0 <= t + eps) lo = mid; else hi = mid - 1;
        }
    } else {
        lo = frame < (long long)tm->n ? (size_t)frame : tm->n - 1;
    }
    return &tm->e[lo];
}

static void be16(unsigned char *p, unsigned v) { p[0] = (unsigned char)(v >> 8); p[1] = (unsigned char)v; }
static void be32(unsigned char *p, unsigned long v) {
    p[0] = (unsigned char)(v >> 24); p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);  p[3] = (unsigned char)v;
}
static void exif_entry(unsigned char *e, unsigned tag, unsigned type,
                       unsigned cnt, unsigned val) {
    be16(e, tag); be16(e + 2, type); be32(e + 4, cnt); be32(e + 8, val);
}
/* 度 -> [deg/1, min/1, sec*10000/10000] */
static void exif_dms(unsigned char *p, double v) {
    double a = v < 0 ? -v : v;
    int deg = (int)a;
    double mf = (a - deg) * 60.0;
    int min = (int)mf;
    unsigned long sec = (unsigned long)((mf - min) * 600000.0 + 0.5);  /* 秒×10⁴ */
    if (sec >= 600000UL) sec = 599999UL;
    be32(p, (unsigned long)deg); be32(p + 4, 1);
    be32(p + 8, (unsigned long)min); be32(p + 12, 1);
    be32(p + 16, sec); be32(p + 20, 10000);
}

/* 把一条遥测写成完整 APP1 段（FF E1 + 长度 + "Exif\0\0" + TIFF/GPS）。
 * 布局固定：header 8 | IFD0(3 项) 42 | Exif IFD(UserComment) 18 |
 * GPS IFD(7 项) 90 | 数据区（度分秒 56B + 描述串 + 注释串）。
 * 返回段总字节数；0 = 无有效坐标，不注入。 */
static int exif_gps_app1(const SrtEntry *se, unsigned char *o, int cap) {
    if (se->lat == 0.0 && se->lon == 0.0) return 0;   /* 无定位不伪造 */
    char desc[200], ucom[160];
    _snprintf(desc, sizeof desc,
              "DJI telemetry: lat %.6f lon %.6f rel_alt %.2f m abs_alt %.2f m",
              se->lat, se->lon, se->rel_alt, se->abs_alt);
    _snprintf(ucom, sizeof ucom, "lat %.6f lon %.6f rel_alt %.2f abs_alt %.2f",
              se->lat, se->lon, se->rel_alt, se->abs_alt);
    int ndesc = (int)strlen(desc) + 1;
    int ncom = 8 + (int)strlen(ucom) + 1;             /* "ASCII\0\0\0" + 文本 */

    int ifd0 = 8, exif_ifd = ifd0 + 42, gps_ifd = exif_ifd + 18;
    int r1 = gps_ifd + 90, r2 = r1 + 24, r3 = r2 + 24;
    int doff = r3 + 8, coff = doff + ((ndesc + 1) & ~1);
    int tiff_end = coff + ((ncom + 1) & ~1);
    int payload = 6 + tiff_end;
    int total = 4 + payload;                          /* FF E1 + 段长 + payload */
    if (total > cap || payload > 65533) return 0;

    memcpy(o, "\xFF\xE1", 2);
    be16(o + 2, (unsigned)(payload + 2));
    memcpy(o + 4, "Exif\0\0", 6);
    unsigned char *t = o + 10;
    memcpy(t, "MM\0*\0\0\0\x08", 8);
    unsigned char *i0 = t + ifd0;
    be16(i0, 3);
    exif_entry(i0 + 2,  0x010E, 2, (unsigned)ndesc, (unsigned long)doff);   /* ImageDescription */
    exif_entry(i0 + 14, 0x8769, 4, 1, (unsigned long)exif_ifd);             /* Exif IFD pointer */
    exif_entry(i0 + 26, 0x8825, 4, 1, (unsigned long)gps_ifd);              /* GPS IFD pointer */
    be32(i0 + 38, 0);
    unsigned char *ie = t + exif_ifd;
    be16(ie, 1);
    exif_entry(ie + 2, 0x9286, 7, (unsigned)ncom, (unsigned long)coff);     /* UserComment */
    be32(ie + 14, 0);
    unsigned char *ig = t + gps_ifd;
    be16(ig, 7);
    exif_entry(ig + 2,  0x0000, 1, 4, 0x02020000);                          /* GPSVersionID inline */
    exif_entry(ig + 14, 0x0001, 2, 2, se->lat >= 0 ? 0x4E000000 : 0x53000000); /* 'N'/'S' */
    exif_entry(ig + 26, 0x0002, 5, 3, (unsigned long)r1);                   /* GPSLatitude */
    exif_entry(ig + 38, 0x0003, 2, 2, se->lon >= 0 ? 0x45000000 : 0x57000000); /* 'E'/'W' */
    exif_entry(ig + 50, 0x0004, 5, 3, (unsigned long)r2);                   /* GPSLongitude */
    exif_entry(ig + 62, 0x0005, 1, 1, se->abs_alt >= 0 ? 0 : 1);            /* AltitudeRef */
    exif_entry(ig + 74, 0x0006, 5, 1, (unsigned long)r3);                   /* GPSAltitude */
    be32(ig + 86, 0);
    double alt = se->abs_alt < 0 ? -se->abs_alt : se->abs_alt;
    be32(t + r3, (unsigned long)(alt * 100.0 + 0.5));
    be32(t + r3 + 4, 100);
    exif_dms(t + r1, se->lat);
    exif_dms(t + r2, se->lon);
    memcpy(t + doff, desc, (size_t)ndesc);
    memcpy(t + coff, "ASCII\0\0\0", 8);
    memcpy(t + coff + 8, ucom, (size_t)ncom - 8);
    return total;
}

/* write one winner jpeg; app1 != NULL 时在 SOI 后注入该段; returns 0 ok */
static int write_jpeg(const wchar_t *dir, long long idx, const uint8_t *d, size_t n,
                      const unsigned char *app1, int app1len) {
    wchar_t path[MAX_PATH];
    _snwprintf(path, MAX_PATH, L"%ls\\%05lld.jpg", dir, idx);
    FILE *f = _wfopen(path, L"wb");
    if (!f) return -1;
    if (app1 && app1len > 0 && n > 2) {
        fwrite(d, 1, 2, f);              /* SOI */
        fwrite(app1, 1, (size_t)app1len, f);
        fwrite(d + 2, 1, n - 2, f);
    } else {
        fwrite(d, 1, n, f);
    }
    fclose(f);
    return 0;
}

/* pass B: re-decode, encode every frame as mjpeg over the pipe, and write
 * only the winners (counted by frame number -- no select expression, whose
 * length ffmpeg's expression parser cannot take for a few hundred frames).
 * maskp != NULL 时挂全分辨率圆形遮罩 + maskedmerge：在那唯一一次编码前
 * 把圆外清黑（无额外解码/编码遍，无二次有损编码损失）。 */
static int pass_b(const Ctx *cx, int track, const Cand *winners, size_t nwin,
                  const wchar_t *outdir, const wchar_t *maskp, const Telem *tm) {
    Cmd c = {0};
    cmd_add(&c, cx->ffmpeg);
    cmd_add(&c, L"-v");
    cmd_add(&c, L"error");
    cmd_add(&c, L"-nostdin");
    if (wcslen(cx->hw)) cmd_add2(&c, L"-hwaccel", cx->hw);
    cmd_add(&c, L"-i");
    cmd_add(&c, cx->input);
    if (maskp) {
        /* 副输入 1fps + framesync 重复末帧：每秒时间轴只解一次遮罩 */
        cmd_add(&c, L"-loop");
        cmd_add(&c, L"1");
        cmd_add(&c, L"-framerate");
        cmd_add(&c, L"1");
        cmd_add(&c, L"-i");
        cmd_add(&c, maskp);
        cmd_add(&c, L"-f");
        cmd_add(&c, L"lavfi");
        cmd_add(&c, L"-i");
        wchar_t blackspec[80];
        _snwprintf(blackspec, 80, L"color=c=black:s=%dx%d:r=1", cx->vw, cx->vh);
        cmd_add(&c, blackspec);
    }
    if (maskp) {
        wchar_t fc[256];
        _snwprintf(fc, 256,
                   L"[0:v:%d]format=gbrp[v];[1:v]format=gbrp[mk];"
                   L"[2:v]format=gbrp[bk];[v][bk][mk]maskedmerge[out]", track);
        cmd_add2(&c, L"-filter_complex", fc);
        cmd_add2(&c, L"-map", L"[out]");
    } else {
        wchar_t mapspec[16]; swprintf(mapspec, 16, L"0:v:%d", track);
        cmd_add2(&c, L"-map", mapspec);
    }
    cmd_add(&c, L"-fps_mode");
    cmd_add(&c, L"passthrough");
    cmd_add(&c, L"-c:v");
    cmd_add(&c, L"mjpeg");
    wchar_t qbuf[8]; swprintf(qbuf, 8, L"%d", cx->jpeg_q);
    cmd_add2(&c, L"-q:v", qbuf);
    cmd_add(&c, L"-f");
    cmd_add(&c, L"mjpeg");
    cmd_add(&c, L"pipe:1");

    Child ch;
    if (spawn(c.s, &ch, 0) != 0) { free(c.s); return -1; }
    free(c.s);
    take_child(ch.pi.hProcess);

    /* split the mjpeg stream: SOI = FF D8 FF ... EOI = FF D9 */
    uint8_t *buf = NULL;
    size_t len = 0, cap = 0, start = 0, pos = 0;
    int in_jpeg = 0;
    long long n = 0;          /* source frame counter, same as pass A */
    size_t wnext = 0;         /* next winner to write (winners sorted) */
    for (;;) {
        if (len == cap) { cap = cap ? cap * 2 : (1 << 20); buf = realloc(buf, cap); }
        DWORD got = 0;
        if (!ReadFile(ch.r, buf + len, (DWORD)(cap - len), &got, NULL) || got == 0)
            break;
        len += got;
        if (cancelled()) break;
        for (;;) {
            if (!in_jpeg) {
                while (pos + 2 < len && !(buf[pos] == 0xFF && buf[pos + 1] == 0xD8 &&
                                           buf[pos + 2] == 0xFF))
                    pos++;
                if (pos + 2 >= len) break;
                start = pos;
                in_jpeg = 1;
                pos += 2;
            } else {
                while (pos + 1 < len && !(buf[pos] == 0xFF && buf[pos + 1] == 0xD9))
                    pos++;
                if (pos + 1 >= len) break;
                size_t end = pos + 2;
                if (wnext < nwin && winners[wnext].idx == n) {
                    unsigned char app1[2048];
                    int alen = 0;
                    if (tm) {
                        const SrtEntry *se = telem_lookup(tm, n);
                        if (se) alen = exif_gps_app1(se, app1, sizeof app1);
                    }
                    if (write_jpeg(outdir, n, buf + start, end - start,
                                   alen ? app1 : NULL, alen) != 0) {
                        LOG("pass B: cannot write jpeg %lld\n", n);
                        free(buf);
                        child_wait(&ch);
                        return -1;
                    }
                    wnext++;
                    if (wnext % 20 == 0) STATUS(g_file_idx, 2, track, (long long)wnext);
                }
                n++;
                pos = end;
                in_jpeg = 0;
                if (wnext == nwin) goto all_written;
            }
        }
        /* drop consumed bytes -- never the in-flight jpeg: while a jpeg is
         * open everything from `start` on is still needed, so only the
         * bytes before it may go. */
        {
            size_t drop = in_jpeg ? start : pos;
            if (drop > 0) {
                memmove(buf, buf + drop, len - drop);
                len -= drop;
                pos -= drop;
                if (in_jpeg) start = 0;
            }
        }
    }
all_written:
    if (wnext != nwin) {
        fwprintf(stderr, L"pass B: wrote %zu of %zu winners (%lld frames seen)\n",
                 wnext, nwin, n);
        free(buf);
        child_wait(&ch);
        return -1;
    }
    /* all winners written: the remaining decode is pointless, kill ffmpeg */
    free(buf);
    TerminateProcess(ch.pi.hProcess, 0);
    child_wait(&ch);
    return 0;
}

/* ------------------------------------------- 边缘涂黑（并入抽帧编码管线） */

/* 鱼眼帧中心圆外涂黑：黑边 + 紧贴黑边的失焦模糊环一并切掉。
 * 圆心=画面中心，保留半径 = pct% × 短边 ÷ 2；同一镜头所有帧同一比例，
 * 宁多勿少（模糊环内侧还有一圈渐进的过渡软带）。涂黑区无纹理，SfM
 * 特征点天然落不进去，等价于给整条建模链喂了 mask。
 *
 * 实现：pass_b 的 ffmpeg 滤镜链里挂全分辨率圆形遮罩 + maskedmerge，
 * 在那唯一一次 mjpeg 编码前把圆外清黑——没有额外的解码/重编码遍，
 * 没有二次有损编码损失，涂黑成本≈内存带宽。 */

typedef struct { uint8_t *buf; size_t n, cap; } MemBuf;

static void mem_write(void *ctx, void *data, int n) {
    MemBuf *m = (MemBuf *)ctx;
    if (m->n + (size_t)n > m->cap) {
        m->cap = (m->n + (size_t)n) * 2;
        m->buf = realloc(m->buf, m->cap);
    }
    memcpy(m->buf + m->n, data, (size_t)n);
    m->n += (size_t)n;
}

/* 生成 %TEMP% 下的圆形遮罩 PNG（灰度：圆内 0 / 圆外 255 / 1.5px 抗锯齿
 * 过渡）。maskedmerge 语义是 mask=255 选第二个输入（黑底），所以圆外
 * 才是 255。按 track 命名防双目两路互相覆盖。返回 _wcsdup 的路径（用完
 * 由调用方 _wunlink + free），失败返回 NULL。 */
static wchar_t *make_mask_png(int w, int h, int pct, int track) {
    wchar_t temp[MAX_PATH * 2];
    wchar_t *path;
    char *p8;
    uint8_t *m;
    FILE *f;
    MemBuf mb = {0};
    double cx = w * 0.5, cy = h * 0.5;
    double r = pct * 0.01 * (double)(w < h ? w : h) * 0.5;
    DWORD n = GetTempPathW(MAX_PATH * 2, temp);
    int x, y, ok;
    if (n == 0 || n >= MAX_PATH * 2) return NULL;
    path = malloc((n + 48) * sizeof(wchar_t));
    if (!path) return NULL;
    _snwprintf(path, n + 47, L"%sosvsharp_mask_%lu_%d.png",
               temp, (unsigned long)GetCurrentProcessId(), track);
    m = malloc((size_t)w * h);
    if (!m) { free(path); return NULL; }
    for (y = 0; y < h; y++) {
        uint8_t *row = m + (size_t)y * w;
        double dy = y + 0.5 - cy, dy2 = dy * dy;
        for (x = 0; x < w; x++) {
            double dx = x + 0.5 - cx;
            double d = sqrt(dx * dx + dy2);
            if (d <= r - 1.5) row[x] = 0;
            else if (d >= r)  row[x] = 255;
            else row[x] = (uint8_t)((d - (r - 1.5)) / 1.5 * 255.0 + 0.5);
        }
    }
    p8 = w_to_utf8(path);
    ok = p8 && stbi_write_png_to_func(mem_write, &mb, w, h, 1, m, w);
    free(p8);
    free(m);
    if (!ok) { free(mb.buf); free(path); return NULL; }
    f = _wfopen(path, L"wb");
    if (!f) { free(mb.buf); free(path); return NULL; }
    ok = mb.n > 0 && fwrite(mb.buf, 1, mb.n, f) == mb.n;
    fclose(f);
    free(mb.buf);
    if (!ok) { _wunlink(path); free(path); return NULL; }
    return path;
}

/* ------------------------------------------------------------------- GUI */

#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "shell32.lib")

#define APP_LOG      (WM_APP + 1)   /* wParam: malloc'd utf8 line */
#define APP_STATUS   (WM_APP + 2)   /* wParam: malloc'd StatusMsg */
#define APP_FDONE    (WM_APP + 3)   /* wParam: malloc'd {idx, rc} */
#define APP_ALLDONE  (WM_APP + 4)
#define APP_TESTDONE (WM_APP + 7)   /* 涂黑测试：对比大图就绪 */

#define IDC_LIST     1001
#define IDC_LOG      1002
#define IDC_E_SKIP   1011
#define IDC_E_KEEP   1012
#define IDC_E_MIN    1013
#define IDC_E_Q      1014
#define IDC_E_OUT    1015
#define IDC_E_MASK   1016
#define IDC_E_DIFF   1017
#define IDC_B_ADD    1021
#define IDC_B_REM    1022
#define IDC_B_CLEAR  1023
#define IDC_B_START  1024
#define IDC_B_STOP   1025
#define IDC_B_BROWSE 1026
#define IDC_B_HELP   1027
#define IDC_B_TEST   1028
#define IDC_PBAR     1031
/* 参数框下方的灰色小提示文本（WM_CTLCOLORSTATIC 里按此 ID 段置灰） */
#define IDC_HINT_SKIP 1041
#define IDC_HINT_KEEP 1042
#define IDC_HINT_MIN  1043
#define IDC_HINT_Q    1044
#define IDC_HINT_MASK 1045
#define IDC_HINT_OUT  1046
#define IDC_HINT_DIFF 1047

typedef struct { int idx, phase, track; long long a; } StatusMsg;
typedef struct { int idx, rc; } DoneMsg;

/* defined in the CLI/main section below */
static wchar_t *find_ffmpeg(const wchar_t *ffmpeg_arg);
static void split_stem(const wchar_t *path, wchar_t *dir, wchar_t *stem);
static int probe_video_streams(const wchar_t *ffmpeg, const wchar_t *input,
                               int *idx, int *ws, int *hs, int maxn,
                               int *nattach, double *fps_out);
static int process_input(const wchar_t *ffmpeg, const wchar_t *input,
                         const wchar_t *outdir, int skip, int keep,
                         double min_score, double diff_th, int min_gap,
                         int max_gap, int track, int thumb, int jpeg_q,
                         int mask_pct, const wchar_t *hw_force, int file_idx);

static HWND g_hwnd, g_list, g_log, g_pbar;
static HFONT g_font, g_font_hint;
static wchar_t (*g_paths)[MAX_PATH];
static int g_npaths, g_cappaths;
static int g_running, g_files_done, g_files_total;
/* 每个文件的"清晰帧"列累计值：cam0/cam1 依次累加，避免后一条轨
 * 把前面的张数覆盖成自己的（曾让单路 MP4 显示"完成 0 张"） */
static int g_cnt_idx = -1;
static long long g_cnt_total;

static void gui_log(void *user, const char *line) {
    (void)user;
    char *dup = _strdup(line);
    PostMessage(g_hwnd, APP_LOG, (WPARAM)dup, 0);
}

static void gui_status(void *user, int idx, int phase, int track, long long a) {
    (void)user;
    StatusMsg *m = malloc(sizeof(*m));
    m->idx = idx; m->phase = phase; m->track = track; m->a = a;
    PostMessage(g_hwnd, APP_STATUS, (WPARAM)m, 0);
}

static void gui_fdone(void *user, int idx, int rc) {
    (void)user;
    DoneMsg *m = malloc(sizeof(*m));
    m->idx = idx; m->rc = rc;
    PostMessage(g_hwnd, APP_FDONE, (WPARAM)m, 0);
}

static void append_log(const char *utf8) {
    /* LOG 管线传的是 UTF-8 字节，必须转宽字符后用 W 版消息写入；
     * 走 A 版会被 ANSI(GBK) 代码页重新解释，中文路径就成了“瀹夊窘涓洂”。 */
    int wn = MultiByteToWideChar(CP_UTF8, 0, utf8, -1, NULL, 0);
    if (wn <= 0) return;
    wchar_t *w = malloc((size_t)wn * sizeof(wchar_t));
    MultiByteToWideChar(CP_UTF8, 0, utf8, -1, w, wn);
    int len = GetWindowTextLengthW(g_log);
    SendMessageW(g_log, EM_SETSEL, len, len);
    SendMessageW(g_log, EM_REPLACESEL, FALSE, (LPARAM)w);
    free(w);
}

static void list_add(const wchar_t *path) {
    if (g_npaths == g_cappaths) {
        g_cappaths = g_cappaths ? g_cappaths * 2 : 64;
        g_paths = realloc(g_paths, sizeof(*g_paths) * (size_t)g_cappaths);
    }
    wcsncpy(g_paths[g_npaths], path, MAX_PATH - 1);
    g_paths[g_npaths][MAX_PATH - 1] = 0;
    const wchar_t *base = wcsrchr(path, L'\\');
    base = base ? base + 1 : path;
    LVITEMW it = {0};
    it.mask = LVIF_TEXT;
    it.iItem = g_npaths;
    it.pszText = (LPWSTR)base;
    ListView_InsertItem(g_list, &it);
    ListView_SetItemText(g_list, g_npaths, 1, (LPWSTR)L"等待");
    ListView_SetItemText(g_list, g_npaths, 2, (LPWSTR)L"");
    g_npaths++;
}

static DWORD WINAPI worker_proc(LPVOID param) {
    wchar_t *outbase = param;      /* L"" = beside each video */
    Hooks hk = {0};
    hk.log = gui_log;
    hk.file_status = gui_status;
    hk.file_done = gui_fdone;
    hk.cancel = &g_cancel;
    g_hk = &hk;

    /* 读取界面参数。各参数的完整语义、取值建议见文件顶部的
     * “参数详解（速查）”注释块。 */
    int translated = 0;
    int skip = GetDlgItemInt(g_hwnd, IDC_E_SKIP, &translated, FALSE); /* 抽帧间隔 */
    if (!translated || skip < 1) skip = DEF_SKIP;
    /* 锐度窗口：留空/非法 → 自动取 skip/2；填 0 → 关闭挑帧
     * （每组固定取第 1 帧，只剩最低清晰度过滤）。 */
    int keep = GetDlgItemInt(g_hwnd, IDC_E_KEEP, &translated, FALSE);
    if (!translated || keep < 0) keep = (int)(0.5 * skip + 0.5); /* empty = auto */
    int q = GetDlgItemInt(g_hwnd, IDC_E_Q, &translated, FALSE);   /* JPEG质量 */
    if (!translated || q < 1 || q > 31) q = DEF_JPEG_Q;
    /* 涂黑半径%：拆解完成后把每张图中心圆外涂黑（切黑边+失焦模糊环）。
     * 留空/非法 = 默认 DEF_MASK；0 = 不涂黑。 */
    int maskpct = GetDlgItemInt(g_hwnd, IDC_E_MASK, &translated, FALSE);
    if (!translated || maskpct < 0 || maskpct > 100) maskpct = DEF_MASK;
    /* 最低清晰度：拉普拉斯方差下限，0 = 不过滤。参考输出目录旁的
     * sharpness_camN.csv 分数分布来定值。 */
    double minscore = 0.0;
    {
        wchar_t buf[64];
        GetDlgItemTextW(g_hwnd, IDC_E_MIN, buf, 64);
        minscore = _wtof(buf);
        if (minscore < 0.0) minscore = 0.0;
    }
    /* 变化阈值：>0 启用动态间隔（画面变化攒够才出图，间隔自动在
     * 4~32 帧间伸缩）；0/留空 = 固定间隔模式。此模式下抽帧间隔/
     * 锐度窗口被忽略。最小/最大出图间隔用编译期默认值（--min-gap/
     * --max-gap 仅命令行可调）。 */
    double diffth = 0.0;
    {
        wchar_t buf[64];
        GetDlgItemTextW(g_hwnd, IDC_E_DIFF, buf, 64);
        diffth = _wtof(buf);
        if (diffth < 0.0) diffth = 0.0;
    }

    wchar_t *ffmpeg = find_ffmpeg(NULL);

    for (int i = 0; i < g_files_total; i++) {
        if (g_cancel) break;
        wchar_t dir[MAX_PATH * 2], stem[MAX_PATH], *outdir;
        split_stem(g_paths[i], dir, stem);
        if (wcslen(outbase) == 0) {
            size_t n = wcslen(dir) + wcslen(stem) + 16;
            outdir = malloc(n * sizeof(wchar_t));
            _snwprintf(outdir, n, L"%ls\\%ls_sharp", dir, stem);
        } else if (g_files_total == 1) {
            outdir = _wcsdup(outbase);
        } else {
            size_t n = wcslen(outbase) + wcslen(stem) + 4;
            outdir = malloc(n * sizeof(wchar_t));
            _snwprintf(outdir, n, L"%ls\\%ls", outbase, stem);
        }
        int rc = process_input(ffmpeg, g_paths[i], outdir, skip, keep, minscore,
                               diffth, DEF_ADAPT_MIN_GAP, DEF_ADAPT_MAX_GAP,
                               -1, DEF_THUMB, q, maskpct, NULL, i);
        free(outdir);
        gui_fdone(NULL, i, rc);
        if (rc == 2) break; /* cancelled */
    }
    free(outbase);
    free(ffmpeg);
    g_hk = NULL;
    PostMessage(g_hwnd, APP_ALLDONE, 0, 0);
    return 0;
}

static void start_batch(void) {
    if (g_running || g_npaths == 0) return;
    g_running = 1;
    g_cancel = 0;
    g_files_done = 0;
    g_files_total = g_npaths;
    g_cnt_idx = -1;
    for (int i = 0; i < g_npaths; i++)
        ListView_SetItemText(g_list, i, 1, (LPWSTR)L"等待");
    SendMessage(g_pbar, PBM_SETPOS, 0, 0);
    EnableWindow(GetDlgItem(g_hwnd, IDC_B_START), FALSE);
    EnableWindow(GetDlgItem(g_hwnd, IDC_B_STOP), TRUE);
    EnableWindow(GetDlgItem(g_hwnd, IDC_B_ADD), FALSE);
    EnableWindow(GetDlgItem(g_hwnd, IDC_B_REM), FALSE);
    EnableWindow(GetDlgItem(g_hwnd, IDC_B_CLEAR), FALSE);
    wchar_t *base = malloc((MAX_PATH * 2) * sizeof(wchar_t));
    GetDlgItemTextW(g_hwnd, IDC_E_OUT, base, MAX_PATH * 2);
    HANDLE th = CreateThread(NULL, 0, worker_proc, base, 0, NULL);
    if (th) CloseHandle(th);
}

static void layout(HWND wnd) {
    RECT rc;
    GetClientRect(wnd, &rc);
    int w = rc.right, h = rc.bottom;
    int x = 10, y = 10;
    MoveWindow(GetDlgItem(wnd, IDC_E_SKIP), x + 62, y, 36, 23, TRUE); x += 110;
    MoveWindow(GetDlgItem(wnd, IDC_E_KEEP), x + 62, y, 36, 23, TRUE); x += 110;
    MoveWindow(GetDlgItem(wnd, IDC_E_DIFF), x + 62, y, 34, 23, TRUE); x += 92;
    MoveWindow(GetDlgItem(wnd, IDC_E_MIN), x + 70, y, 44, 23, TRUE); x += 126;
    MoveWindow(GetDlgItem(wnd, IDC_E_Q), x + 64, y, 30, 23, TRUE); x += 92;
    MoveWindow(GetDlgItem(wnd, IDC_E_MASK), x + 44, y, 40, 23, TRUE); x += 92;
    MoveWindow(GetDlgItem(wnd, IDC_E_OUT), x + 64, y, w - x - 88, 23, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_BROWSE), w - 78, y, 68, 23, TRUE);
    y += 74; /* 跳过参数框下方的三行提示文本 */
    MoveWindow(GetDlgItem(wnd, IDC_B_ADD), 10, y, 90, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_REM), 105, y, 90, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_CLEAR), 200, y, 60, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_HELP), 265, y, 90, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_TEST), 360, y, 100, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_START), w - 180, y, 80, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_STOP), w - 90, y, 80, 26, TRUE);
    MoveWindow(g_pbar, 470, y + 4, w - 660, 18, TRUE);
    y += 36;
    int lh = (h - y) * 45 / 100;
    MoveWindow(g_list, 10, y, w - 20, lh, TRUE);
    MoveWindow(g_log, 10, y + lh + 6, w - 20, h - y - lh - 14, TRUE);
}

static HWND make_ctl(HWND parent, const wchar_t *cls, const wchar_t *text,
                     DWORD style, int id) {
    HWND h = CreateWindowW(cls, text, WS_CHILD | WS_VISIBLE | style,
                           0, 0, 10, 10, parent, (HMENU)(INT_PTR)id,
                           GetModuleHandleW(NULL), NULL);
    SendMessageW(h, WM_SETFONT, (WPARAM)g_font, TRUE);
    return h;
}

static void add_label(HWND parent, const wchar_t *text, int x, int w) {
    HWND l = make_ctl(parent, L"STATIC", text, SS_LEFT, 0);
    SetWindowPos(l, 0, x, 13, w, 18, SWP_NOZORDER);
}

/* 参数框下方的小号灰色提示文本；颜色由 WM_CTLCOLORSTATIC 按 ID 段处理 */
static void add_hint(HWND parent, const wchar_t *text, int x, int w, int id) {
    HWND l = make_ctl(parent, L"STATIC", text, SS_LEFT, id);
    SendMessageW(l, WM_SETFONT, (WPARAM)g_font_hint, TRUE);
    SetWindowPos(l, 0, x, 34, w, 48, SWP_NOZORDER);
}

static void add_files_dialog(HWND owner) {
    wchar_t *buf = malloc(32768 * sizeof(wchar_t));
    buf[0] = 0;
    OPENFILENAMEW ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = owner;
    ofn.lpstrFilter = L"视频文件\0*.osv;*.mp4;*.mov;*.mkv;*.insv;*.avi\0所有文件\0*.*\0";
    ofn.lpstrFile = buf;
    ofn.nMaxFile = 32768;
    ofn.Flags = OFN_ALLOWMULTISELECT | OFN_EXPLORER | OFN_FILEMUSTEXIST;
    ofn.lpstrTitle = L"选择一个或多个视频";
    if (GetOpenFileNameW(&ofn)) {
        wchar_t *p = buf + wcslen(buf) + 1;
        if (*p == 0) {
            list_add(buf);
        } else {
            wchar_t path[MAX_PATH * 2];
            while (*p) {
                _snwprintf(path, MAX_PATH * 2, L"%ls\\%ls", buf, p);
                list_add(path);
                p += wcslen(p) + 1;
            }
        }
    }
    free(buf);
}

static void browse_outdir(HWND owner) {
    BROWSEINFOW bi = {0};
    bi.hwndOwner = owner;
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    bi.lpszTitle = L"选择输出根目录（留空 = 输出到各视频旁边）";
    PIDLIST_ABSOLUTE pidl = SHBrowseForFolderW(&bi);
    if (pidl) {
        wchar_t dir[MAX_PATH];
        if (SHGetPathFromIDListW(pidl, dir))
            SetDlgItemTextW(owner, IDC_E_OUT, dir);
        CoTaskMemFree((void *)pidl);
    }
}

static void remove_selected(void) {
    for (int i = g_npaths - 1; i >= 0; i--) {
        if (ListView_GetItemState(g_list, i, LVIS_SELECTED) & LVIS_SELECTED) {
            ListView_DeleteItem(g_list, i);
            memmove(&g_paths[i], &g_paths[i + 1],
                    sizeof(*g_paths) * (size_t)(g_npaths - i - 1));
            g_npaths--;
        }
    }
}

/* ------------------------------------------------------------ preview ----
 * Double-click a list row: 12 thumbnails sampled at evenly spaced
 * timestamps (-ss + one decoded frame each, so it is fast at any
 * resolution), composed into an in-memory bitmap -- no temp files. */

#define _PV_WSTR2(x) L#x
#define _PV_WSTR(x) _PV_WSTR2(x)

#define PV_COLS 4
#define PV_ROWS 3
#define PV_TW   200
#define PV_TH   150
#define PV_NT   (PV_COLS * PV_ROWS)

#define APP_PVPROG (WM_APP + 5)   /* wParam: tiles done */
#define APP_PVDONE (WM_APP + 6)   /* wParam: 1 = complete */

typedef struct {
    wchar_t path[MAX_PATH * 2];
    wchar_t name[MAX_PATH];
    wchar_t info[256];        /* info line under the title bar          */
    uint8_t *grid;            /* (PV_COLS*PV_TW)x(PV_ROWS*PV_TH) bgr24  */
    int done, ready;
    volatile int cancel;
} Preview;

static Preview g_pv;
static HWND g_hwnd_pv;
static HANDLE g_pv_thread;
static HANDLE g_pv_child;
static wchar_t *g_ffmpeg;     /* cached find_ffmpeg() result            */

static void pv_kill_child(void) {
    HANDLE ch = (HANDLE)InterlockedExchangePointer(
        (void *volatile *)&g_pv_child, NULL);
    if (ch) TerminateProcess(ch, 1);
}

/* run ffmpeg -i <path> (no output) and parse duration/streams from stderr */
static int pv_probe_info(const wchar_t *path, double *dur, int *vw, int *vh,
                         int *nvstreams) {
    Cmd c = {0};
    cmd_add(&c, g_ffmpeg ? g_ffmpeg : L"ffmpeg");
    cmd_add(&c, L"-nostdin");
    cmd_add(&c, L"-i");
    cmd_add(&c, path);
    Child ch;
    if (spawn(c.s, &ch, 2) != 0) { free(c.s); return -1; }
    free(c.s);
    pv_kill_child();
    InterlockedExchangePointer((void *volatile *)&g_pv_child, ch.pi.hProcess);

    size_t len = 0, cap = 65536;
    char *buf = malloc(cap);
    for (;;) {
        if (len == cap) { cap *= 2; buf = realloc(buf, cap); }
        DWORD got = 0;
        if (!ReadFile(ch.r, buf + len, (DWORD)(cap - len), &got, NULL) || got == 0)
            break;
        len += got;
    }
    buf[len < cap ? len : cap - 1] = 0;
    child_wait(&ch);

    int ok = 0;
    *dur = 0; *vw = *vh = 0; *nvstreams = 0;
    const char *p = strstr(buf, "Duration: ");
    if (p) {
        int h = 0, m = 0;
        double s = 0;
        if (sscanf(p + 10, "%d:%d:%lf", &h, &m, &s) == 3) {
            *dur = h * 3600.0 + m * 60.0 + s;
            ok = 1;
        }
    }
    p = buf;
    while ((p = strstr(p, "Video: ")) != NULL) {
        (*nvstreams)++;
        if (*vw == 0) {
            const char *q = p;
            while (*q && *q != '\n') {
                int w, h;
                if (sscanf(q, ", %dx%d", &w, &h) == 2 && w > 0) {
                    *vw = w; *vh = h;
                    break;
                }
                q++;
            }
        }
        p += 7;
    }
    free(buf);
    return ok ? 0 : -1;
}

/* fetch one 200x150 bgr24 thumbnail at time t; 0 ok */
static int pv_fetch_tile(const wchar_t *path, double t, uint8_t *dst) {
    Cmd c = {0};
    cmd_add(&c, g_ffmpeg ? g_ffmpeg : L"ffmpeg");
    cmd_add(&c, L"-nostdin");
    cmd_add(&c, L"-v");
    cmd_add(&c, L"error");
    wchar_t ssbuf[32];
    swprintf(ssbuf, 32, L"%.3f", t);
    cmd_add2(&c, L"-ss", ssbuf);
    cmd_add(&c, L"-i");
    cmd_add(&c, path);
    cmd_add(&c, L"-map");
    cmd_add(&c, L"0:v:0");
    cmd_add(&c, L"-frames:v");
    cmd_add(&c, L"1");
    cmd_add(&c, L"-fps_mode");
    cmd_add(&c, L"passthrough");
    cmd_add(&c, L"-vf");
    cmd_add(&c, L"scale=" _PV_WSTR(PV_TW) L":" _PV_WSTR(PV_TH)
                  L":force_original_aspect_ratio=decrease,pad="
                  _PV_WSTR(PV_TW) L":" _PV_WSTR(PV_TH)
                  L":(ow-iw)/2:(oh-ih)/2,format=bgr24");
    cmd_add(&c, L"-f");
    cmd_add(&c, L"rawvideo");
    cmd_add(&c, L"pipe:1");
    Child ch;
    if (spawn(c.s, &ch, 1) != 0) { free(c.s); return -1; }
    free(c.s);
    pv_kill_child();
    InterlockedExchangePointer((void *volatile *)&g_pv_child, ch.pi.hProcess);

    int ok = read_exact(ch.r, dst, (size_t)PV_TW * PV_TH * 3);
    child_wait(&ch);
    return ok ? 0 : -1;
}

/* DJI's preview trick: the .LRF beside an .OSV is a low-res stitched
 * proxy (2048x1024 h264) -- preview from it when it exists. */
static wchar_t *pv_lrf_path(const wchar_t *video) {
    const wchar_t *dot = wcsrchr(video, L'.');
    const wchar_t *slash = wcsrchr(video, L'\\');
    if (!dot || (slash && dot < slash)) return NULL;
    static const wchar_t *exts[] = { L"LRF", L"lrf", NULL };
    for (int i = 0; exts[i]; i++) {
        size_t n = (size_t)(dot - video) + 8;
        wchar_t *cand = malloc(n * sizeof(wchar_t));
        _snwprintf(cand, n, L"%.*ls.%ls", (int)(dot - video), video, exts[i]);
        DWORD attr = GetFileAttributesW(cand);
        if (attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY))
            return cand;
        free(cand);
    }
    return NULL;
}

static DWORD WINAPI pv_worker(LPVOID param) {
    (void)param;
    wchar_t *lrf = pv_lrf_path(g_pv.path);
    const wchar_t *pvs = lrf ? lrf : g_pv.path;   /* preview source */
    double dur = 0;
    int vw = 0, vh = 0, nv = 0;
    if (pv_probe_info(pvs, &dur, &vw, &vh, &nv) != 0) {
        wcscpy(g_pv.info, L"无法读取该文件（不是视频或已损坏）");
        if (g_hwnd_pv) PostMessage(g_hwnd_pv, APP_PVDONE, 0, 0);
        return 0;
    }
    if (dur <= 0) dur = 60.0;

    wchar_t timebuf[32];
    _snwprintf(timebuf, 32, L"%d:%05.2f", (int)(dur / 60), dur - 60 * (int)(dur / 60));
    _snwprintf(g_pv.info, 256,
               L"时长 %s   分辨率 %dx%d   %s%s   双击其他行可切换预览",
               timebuf, vw, vh,
               nv >= 2 ? L"双目全景（cam0 + cam1）" : L"单路视频",
               lrf ? L"   [LRF 低清代理]" : L"");

    free(g_pv.grid);
    g_pv.grid = NULL;
    g_pv.grid = calloc((size_t)PV_TW * PV_TH * 3, PV_COLS * PV_ROWS);
    g_pv.done = 0;
    for (int i = 0; i < PV_NT; i++) {
        if (g_pv.cancel) break;
        double t = dur * (i + 0.5) / PV_NT;
        uint8_t *tile = malloc((size_t)PV_TW * PV_TH * 3);
        if (pv_fetch_tile(pvs, t, tile) == 0) {
            int col = i % PV_COLS, row = i / PV_COLS;
            uint8_t *dst = g_pv.grid +
                (((size_t)row * PV_TH) * PV_COLS * PV_TW + (size_t)col * PV_TW) * 3;
            /* copy row by row: tile rows are contiguous, grid rows are
             * PV_COLS*PV_TW*3 bytes apart */
            for (int y = 0; y < PV_TH; y++)
                memcpy(dst + (size_t)y * PV_COLS * PV_TW * 3,
                       tile + (size_t)y * PV_TW * 3,
                       (size_t)PV_TW * 3);
            g_pv.done = i + 1;
            if (g_hwnd_pv) PostMessage(g_hwnd_pv, APP_PVPROG, i + 1, 0);
        }
        free(tile);
    }
    if (g_hwnd_pv) PostMessage(g_hwnd_pv, APP_PVDONE, 1, 0);
    free(lrf);
    return 0;
}

static void pv_paint(HWND wnd) {
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(wnd, &ps);
    RECT rc;
    GetClientRect(wnd, &rc);
    FillRect(dc, &rc, (HBRUSH)GetStockObject(WHITE_BRUSH));
    SetBkMode(dc, TRANSPARENT);
    SelectObject(dc, g_font);

    RECT rinfo = { 10, 8, rc.right - 10, 34 };
    DrawTextW(dc, g_pv.name, -1, &rinfo,
              DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
    RECT rinfo2 = { 10, 34, rc.right - 10, 56 };
    DrawTextW(dc, g_pv.info, -1, &rinfo2,
              DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);

    int gx = 8, gy = 62;
    int gw = PV_COLS * PV_TW, gh = PV_ROWS * PV_TH;
    if (g_pv.grid) {
        BITMAPINFO bi;
        ZeroMemory(&bi, sizeof(bi));
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = gw;
        bi.bmiHeader.biHeight = -gh; /* top-down */
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 24;
        bi.bmiHeader.biSizeImage = (DWORD)(gw * gh * 3);
        StretchDIBits(dc, gx, gy, gw, gh, 0, 0, gw, gh, g_pv.grid, &bi,
                      DIB_RGB_COLORS, SRCCOPY);
    }
    if (!g_pv.ready) {
        wchar_t st[64];
        _snwprintf(st, 64, L"正在生成预览…  %d/%d", g_pv.done, PV_NT);
        RECT rcst = { gx, gy + gh / 2 - 12, gx + gw, gy + gh / 2 + 12 };
        SetTextColor(dc, RGB(90, 90, 90));
        DrawTextW(dc, st, -1, &rcst, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    }
    EndPaint(wnd, &ps);
}

static LRESULT CALLBACK pv_wndproc(HWND wnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_PAINT: pv_paint(wnd); return 0;
    case WM_ERASEBKGND: return 1;
    case APP_PVPROG:
        g_pv.done = (int)wp;
        InvalidateRect(wnd, NULL, FALSE);
        return 0;
    case APP_PVDONE:
        g_pv.ready = 1;
        InvalidateRect(wnd, NULL, FALSE);
        return 0;
    case WM_DESTROY:
        g_pv.cancel = 1;
        pv_kill_child();
        g_hwnd_pv = NULL;
        return 0;
    }
    return DefWindowProcW(wnd, msg, wp, lp);
}

static void start_preview(int idx) {
    /* cancel a previous preview generation and let it finish */
    if (g_pv_thread) {
        g_pv.cancel = 1;
        pv_kill_child();
        WaitForSingleObject(g_pv_thread, 10000);
        CloseHandle(g_pv_thread);
        g_pv_thread = NULL;
    }
    g_pv.cancel = 0;
    g_pv.done = 0;
    g_pv.ready = 0;
    wcscpy(g_pv.path, g_paths[idx]);
    const wchar_t *base = wcsrchr(g_pv.path, L'\\');
    base = base ? base + 1 : g_pv.path;
    wcsncpy(g_pv.name, base, 255); g_pv.name[255] = 0;
    wcscpy(g_pv.info, L"");

    if (!g_hwnd_pv) {
        RECT wr = { 0, 0, PV_COLS * PV_TW + 16, PV_ROWS * PV_TH + 70 };
        AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
        g_hwnd_pv = CreateWindowW(L"OsvSharpPreview",
                                  L"视频预览",
                                  WS_OVERLAPPEDWINDOW,
                                  CW_USEDEFAULT, CW_USEDEFAULT,
                                  wr.right - wr.left, wr.bottom - wr.top,
                                  g_hwnd, NULL, GetModuleHandleW(NULL), NULL);
        ShowWindow(g_hwnd_pv, SW_SHOW);
    } else {
        SetWindowTextW(g_hwnd_pv, L"视频预览");
        ShowWindow(g_hwnd_pv, SW_SHOW);
        SetForegroundWindow(g_hwnd_pv);
    }
    InvalidateRect(g_hwnd_pv, NULL, TRUE);
    UpdateWindow(g_hwnd_pv);
    g_pv_thread = CreateThread(NULL, 0, pv_worker, NULL, 0, NULL);
}

/* ------------------------------------------------------------ 涂黑测试 ----
 * 提取所选视频每一路的第一帧，按 85~99% 各涂黑一版拼成对比大图展示；
 * 点击任意一格即把主窗口「涂黑%」输入框设为该值，由用户定涂黑范围。 */

#define TEST_COLS   5
#define TEST_NPCT   15              /* 85..99 */
#define TEST_TW     600             /* 单格宽（px） */
#define TEST_HEADER 40              /* 每路区段标题条高度 */
#define TEST_GAP    6

typedef struct {
    uint8_t *sheet;       /* bgr24 对比大图 */
    int w, h;
    int tile_h;           /* 单格高（按源帧纵横比） */
    int sec_h;            /* 每路区段高度 */
    int tracks;           /* 1 = 单路, 2 = 双目 */
} TestSheet;

static TestSheet g_test;
static HWND g_hwnd_test;
static HANDLE g_test_thread;
static int g_test_scroll, g_test_failed;
static double g_test_scale = 1.0;

static void test_free_sheet(void) {
    free(g_test.sheet);
    memset(&g_test, 0, sizeof(g_test));
}

/* 与 pass_b 滤镜同一几何：圆心=画面中心，保留半径=pct%×短边÷2，1.5px AA */
static void mask_circle_bgr(uint8_t *img, int w, int h, int pct) {
    double cx = w * 0.5, cy = h * 0.5;
    double r = pct * 0.01 * (double)(w < h ? w : h) * 0.5;
    size_t rb = (size_t)w * 3;
    for (int y = 0; y < h; y++) {
        uint8_t *row = img + (size_t)y * rb;
        double dy = y + 0.5 - cy, dy2 = dy * dy;
        for (int x = 0; x < w; x++) {
            double dx = x + 0.5 - cx;
            double d = sqrt(dx * dx + dy2);
            double a = (d <= r - 1.5) ? 1.0 : (d >= r) ? 0.0 : (r - d) / 1.5;
            uint8_t *px = row + (size_t)x * 3;
            px[0] = (uint8_t)(px[0] * a);
            px[1] = (uint8_t)(px[1] * a);
            px[2] = (uint8_t)(px[2] * a);
        }
    }
}

static void downscale_bgr(const uint8_t *src, int sw, int sh,
                          uint8_t *dst, int dw, int dh, int stride) {
    for (int dy = 0; dy < dh; dy++) {
        int y0 = (int)((long long)dy * sh / dh);
        int y1 = (int)((long long)(dy + 1) * sh / dh);
        if (y1 <= y0) y1 = y0 + 1;
        uint8_t *out = dst + (size_t)dy * stride;
        for (int dx = 0; dx < dw; dx++) {
            int x0 = (int)((long long)dx * sw / dw);
            int x1 = (int)((long long)(dx + 1) * sw / dw);
            if (x1 <= x0) x1 = x0 + 1;
            unsigned b = 0, g = 0, r = 0;
            for (int y = y0; y < y1; y++) {
                const uint8_t *row = src + ((size_t)y * sw + x0) * 3;
                for (int x = x0; x < x1; x++, row += 3) {
                    b += row[0]; g += row[1]; r += row[2];
                }
            }
            unsigned n = (unsigned)((y1 - y0) * (x1 - x0));
            out[dx * 3 + 0] = (uint8_t)(b / n);
            out[dx * 3 + 1] = (uint8_t)(g / n);
            out[dx * 3 + 2] = (uint8_t)(r / n);
        }
    }
}

static DWORD WINAPI test_worker(LPVOID argp) {
    wchar_t *path = (wchar_t *)argp;
    wchar_t *ffmpeg = find_ffmpeg(NULL);
    TestSheet ts;
    memset(&ts, 0, sizeof(ts));
    int idx[4], ws[4], hs[4], nattach = 0;
    int nreal = probe_video_streams(ffmpeg, path, idx, ws, hs, 4, &nattach, NULL);
    int ntracks = nreal > 0 ? (nreal > 2 ? 2 : nreal) : 1;
    int failed = 0;
    if (nreal <= 0 || ws[0] <= 0 || hs[0] <= 0) {
        LOG("[涂黑测试] 无法从文件头解析视频流分辨率\n");
        failed = 1;
    } else {
        ts.tile_h = (int)((double)TEST_TW * hs[0] / ws[0] + 0.5);
        ts.w = TEST_COLS * TEST_TW;
        ts.sec_h = TEST_HEADER + 3 * ts.tile_h + 2 * TEST_GAP;
        ts.h = ts.sec_h * ntracks;
        ts.tracks = ntracks;
        ts.sheet = calloc((size_t)ts.w * ts.h, 3);
        if (!ts.sheet) { LOG("[涂黑测试] 内存不足\n"); failed = 1; }
    }
    for (int ti = 0; ti < ntracks && !failed; ti++) {
        char b[160];
        sprintf(b, "[涂黑测试] 提取 cam%d 第一帧（%dx%d）…\n", ti, ws[ti], hs[ti]);
        LOG("%s", b);
        Cmd c = {0};
        cmd_add(&c, ffmpeg);
        cmd_add(&c, L"-v"); cmd_add(&c, L"error"); cmd_add(&c, L"-nostdin");
        cmd_add(&c, L"-i"); cmd_add(&c, path);
        wchar_t mapspec[16]; swprintf(mapspec, 16, L"0:v:%d",
                                      nreal > 0 ? idx[ti] : ti);
        cmd_add2(&c, L"-map", mapspec);
        cmd_add(&c, L"-frames:v"); cmd_add(&c, L"1");
        cmd_add(&c, L"-f"); cmd_add(&c, L"rawvideo");
        cmd_add(&c, L"-pix_fmt"); cmd_add(&c, L"bgr24");
        cmd_add(&c, L"pipe:1");
        Child ch;
        if (spawn(c.s, &ch, 1) != 0) {
            free(c.s);
            LOG("[涂黑测试] 无法启动 ffmpeg\n");
            failed = 1;
            break;
        }
        free(c.s);
        size_t need = (size_t)ws[ti] * hs[ti] * 3;
        uint8_t *frame = malloc(need);
        if (!frame) { child_wait(&ch); LOG("[涂黑测试] 内存不足\n"); failed = 1; break; }
        int ok = read_exact(ch.r, frame, need);
        child_wait(&ch);
        if (!ok) { free(frame); LOG("[涂黑测试] 第一帧解码失败\n"); failed = 1; break; }
        for (int k = 0; k < TEST_NPCT; k++) {
            int pct = 85 + k;
            uint8_t *work = malloc(need);
            if (!work) { LOG("[涂黑测试] 内存不足\n"); failed = 1; break; }
            memcpy(work, frame, need);   /* 每档都从干净帧出发：大半径需要小半径已黑的像素 */
            mask_circle_bgr(work, ws[ti], hs[ti], pct);
            int col = k % TEST_COLS, row = k / TEST_COLS;
            uint8_t *dst = ts.sheet +
                ((size_t)(ti * ts.sec_h + TEST_HEADER + row * (ts.tile_h + TEST_GAP)) * ts.w
                 + (size_t)col * TEST_TW) * 3;
            downscale_bgr(work, ws[ti], hs[ti], dst, TEST_TW, ts.tile_h, ts.w * 3);
            free(work);
        }
        free(frame);
    }
    free(ffmpeg);
    free(path);
    if (failed) {
        free(ts.sheet);
        g_test_failed = 1;
    } else {
        test_free_sheet();
        g_test = ts;
        g_test_failed = 0;
    }
    PostMessage(g_hwnd, APP_TESTDONE, 0, 0);
    return 0;
}

static void test_clamp_scroll(HWND wnd) {
    RECT rc;
    GetClientRect(wnd, &rc);
    double scale = (double)(rc.right > 0 ? rc.right : 1) / g_test.w;
    if (scale > 1.0) scale = 1.0;
    g_test_scale = scale;
    int disp_h = (int)(g_test.h * scale + 0.5);
    int maxpos = disp_h - rc.bottom;
    if (maxpos < 0) maxpos = 0;
    if (g_test_scroll > maxpos) g_test_scroll = maxpos;
    if (g_test_scroll < 0) g_test_scroll = 0;
}

static LRESULT CALLBACK test_wndproc(HWND wnd, UINT m, WPARAM wp, LPARAM lp) {
    switch (m) {
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(wnd, &ps);
        RECT rc;
        GetClientRect(wnd, &rc);
        FillRect(dc, &rc, (HBRUSH)GetStockObject(BLACK_BRUSH));
        SetBkMode(dc, TRANSPARENT);
        if (g_test.sheet && g_test.w > 0) {
            test_clamp_scroll(wnd);
            double scale = g_test_scale;
            int disp_w = (int)(g_test.w * scale + 0.5);
            int disp_h = (int)(g_test.h * scale + 0.5);
            SCROLLINFO si;
            ZeroMemory(&si, sizeof(si));
            si.cbSize = sizeof(si);
            si.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;
            si.nMin = 0; si.nMax = disp_h;
            si.nPage = (UINT)(rc.bottom + 1);
            if (si.nPage > (UINT)disp_h + 1) si.nPage = (UINT)disp_h + 1;
            si.nPos = g_test_scroll;
            SetScrollInfo(wnd, SB_VERT, &si, TRUE);
            BITMAPINFO bi;
            ZeroMemory(&bi, sizeof(bi));
            bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
            bi.bmiHeader.biWidth = g_test.w;
            bi.bmiHeader.biHeight = -g_test.h;
            bi.bmiHeader.biPlanes = 1;
            bi.bmiHeader.biBitCount = 24;
            SetStretchBltMode(dc, COLORONCOLOR);
            StretchDIBits(dc, 0, -g_test_scroll, disp_w, disp_h,
                          0, 0, g_test.w, g_test.h, g_test.sheet, &bi,
                          DIB_RGB_COLORS, SRCCOPY);
            int fh = (int)(TEST_HEADER * scale * 0.55);
            if (fh < 13) fh = 13;
            if (fh > 44) fh = 44;
            HFONT f = CreateFontW(-fh, 0, 0, 0, FW_SEMIBOLD, 0, 0, 0,
                                  DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0,
                                  L"Microsoft YaHei UI");
            HGDIOBJ of = SelectObject(dc, f);
            SetTextColor(dc, RGB(0, 255, 80));
            wchar_t buf[16];
            for (int t = 0; t < g_test.tracks; t++) {
                _snwprintf(buf, 16, L"cam%d", t);
                TextOutW(dc, 10, (int)(t * g_test.sec_h * scale) - g_test_scroll + 4,
                         buf, lstrlenW(buf));
                for (int k = 0; k < TEST_NPCT; k++) {
                    int col = k % TEST_COLS, row = k / TEST_COLS;
                    int x = (int)(col * TEST_TW * scale) + 6;
                    int y = (int)((t * g_test.sec_h + TEST_HEADER + row * (g_test.tile_h + TEST_GAP)) * scale)
                            - g_test_scroll + 4;
                    _snwprintf(buf, 16, L"%d%%", 85 + k);
                    TextOutW(dc, x, y, buf, lstrlenW(buf));
                }
            }
            SelectObject(dc, of);
            DeleteObject(f);
        } else {
            SetTextColor(dc, RGB(190, 190, 190));
            RECT rc2 = { 0, 0, rc.right, rc.bottom };
            DrawTextW(dc, g_test_failed ? L"生成失败（见主窗口日志）"
                                        : L"正在生成对比图…",
                      -1, &rc2, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        }
        EndPaint(wnd, &ps);
        return 0;
    }
    case WM_SIZE:
        InvalidateRect(wnd, NULL, FALSE);
        return 0;
    case WM_VSCROLL: {
        RECT rc;
        GetClientRect(wnd, &rc);
        int line = rc.bottom / 12; if (line < 60) line = 60;
        switch (LOWORD(wp)) {
        case SB_LINEDOWN: g_test_scroll += line; break;
        case SB_LINEUP:   g_test_scroll -= line; break;
        case SB_PAGEDOWN: g_test_scroll += rc.bottom - line; break;
        case SB_PAGEUP:   g_test_scroll -= rc.bottom - line; break;
        case SB_THUMBTRACK: case SB_THUMBPOSITION: g_test_scroll = HIWORD(wp); break;
        case SB_BOTTOM: g_test_scroll = 0x7fffffff; break;
        case SB_TOP:    g_test_scroll = 0; break;
        default: return 0;
        }
        test_clamp_scroll(wnd);
        InvalidateRect(wnd, NULL, FALSE);
        return 0;
    }
    case WM_MOUSEWHEEL: {
        int delta = (short)HIWORD(wp);
        RECT rc;
        GetClientRect(wnd, &rc);
        int line = rc.bottom / 12; if (line < 60) line = 60;
        g_test_scroll -= delta / 120 * line;
        test_clamp_scroll(wnd);
        InvalidateRect(wnd, NULL, FALSE);
        return 0;
    }
    case WM_LBUTTONDOWN: {
        if (!g_test.sheet) return 0;
        double scale = g_test_scale;
        double ix = (double)(SHORT)LOWORD(lp) / scale;
        double iy = (double)((SHORT)HIWORD(lp) + g_test_scroll) / scale;
        for (int t = 0; t < g_test.tracks; t++) {
            for (int k = 0; k < TEST_NPCT; k++) {
                int col = k % TEST_COLS, row = k / TEST_COLS;
                double x0 = (double)col * TEST_TW;
                double y0 = t * g_test.sec_h + TEST_HEADER + row * (g_test.tile_h + TEST_GAP);
                if (ix >= x0 && ix < x0 + TEST_TW && iy >= y0 && iy < y0 + g_test.tile_h) {
                    wchar_t buf[16];
                    char b[128];
                    wsprintfW(buf, L"%d", 85 + k);
                    SetDlgItemTextW(g_hwnd, IDC_E_MASK, buf);
                    sprintf(b, "[涂黑测试] 已把「涂黑%%」设为 %d（点「开始提取」生效）\n", 85 + k);
                    gui_log(NULL, b);
                    return 0;
                }
            }
        }
        return 0;
    }
    case WM_DESTROY:
        g_hwnd_test = NULL;
        return 0;
    }
    return DefWindowProcW(wnd, m, wp, lp);
}

static void start_mask_test(void) {
    if (g_npaths == 0) {
        MessageBoxW(g_hwnd, L"先往列表里添加视频文件", L"涂黑测试", MB_ICONINFORMATION);
        return;
    }
    if (g_test_thread) {
        MessageBoxW(g_hwnd, L"涂黑测试正在进行中…", L"涂黑测试", MB_ICONINFORMATION);
        return;
    }
    int sel = ListView_GetNextItem(g_list, -1, LVNI_SELECTED);
    int item = sel >= 0 ? sel : 0;
    wchar_t *path = _wcsdup(g_paths[item]);
    if (!g_hwnd_test) {
        RECT wr = { 0, 0, 1560, 1060 };
        AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
        g_hwnd_test = CreateWindowW(L"OsvSharpTest",
                                    L"涂黑测试 — 点击任意一格，把「涂黑%」设为该值（85~99）",
                                    WS_OVERLAPPEDWINDOW,
                                    CW_USEDEFAULT, CW_USEDEFAULT,
                                    wr.right - wr.left, wr.bottom - wr.top,
                                    g_hwnd, NULL, GetModuleHandleW(NULL), NULL);
        ShowWindow(g_hwnd_test, SW_SHOW);
    } else {
        ShowWindow(g_hwnd_test, SW_SHOW);
        SetForegroundWindow(g_hwnd_test);
    }
    test_free_sheet();
    g_test_failed = 0;
    g_test_scroll = 0;
    InvalidateRect(g_hwnd_test, NULL, TRUE);
    g_test_thread = CreateThread(NULL, 0, test_worker, path, 0, NULL);
}

static LRESULT CALLBACK wndproc(HWND wnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE: {
        g_font = CreateFontW(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                             DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0,
                             L"Microsoft YaHei UI");
        g_font_hint = CreateFontW(-12, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                                  DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0,
                                  L"Microsoft YaHei UI");
        add_label(wnd, L"抽帧间隔", 10, 58);
        add_label(wnd, L"锐度窗口", 120, 58);
        add_label(wnd, L"变化阈值", 230, 58);
        add_label(wnd, L"最低清晰度", 322, 66);
        add_label(wnd, L"JPEG质量", 448, 60);
        add_label(wnd, L"涂黑%", 540, 40);
        add_label(wnd, L"输出目录", 632, 60);
        add_hint(wnd, L"每N帧出1张\n（≈帧率÷N）", 10, 108, IDC_HINT_SKIP);
        add_hint(wnd, L"组末N帧挑最清晰\n留空=间隔÷2\n0=不挑", 120, 108, IDC_HINT_KEEP);
        add_hint(wnd, L"画面变了才出图\n0=固定间隔\n8≈默认密度", 230, 88, IDC_HINT_DIFF);
        add_hint(wnd, L"胜者分数低于此值丢弃\n0=不过滤；阈值参考\n旁 sharpness_cam0.csv",
                 322, 124, IDC_HINT_MIN);
        add_hint(wnd, L"越小越清晰\n2≈高质量", 448, 90, IDC_HINT_Q);
        add_hint(wnd, L"抽帧时圆外涂黑(黑边+模糊环)\n越小切越多；仅双目OSV\n0=不涂黑", 540, 86, IDC_HINT_MASK);
        add_hint(wnd, L"留空=输出到各视频旁\n<视频名>_sharp\\", 632, 260, IDC_HINT_OUT);
        make_ctl(wnd, L"EDIT", L"8", WS_BORDER | ES_AUTOHSCROLL, IDC_E_SKIP);
        make_ctl(wnd, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL, IDC_E_KEEP);
        make_ctl(wnd, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL, IDC_E_DIFF);
        make_ctl(wnd, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL, IDC_E_MIN);
        make_ctl(wnd, L"EDIT", L"2", WS_BORDER | ES_AUTOHSCROLL, IDC_E_Q);
        make_ctl(wnd, L"EDIT", L"95", WS_BORDER | ES_AUTOHSCROLL, IDC_E_MASK);
        make_ctl(wnd, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL, IDC_E_OUT);
        make_ctl(wnd, L"BUTTON", L"浏览…", 0, IDC_B_BROWSE);
        make_ctl(wnd, L"BUTTON", L"参数说明", 0, IDC_B_HELP);
        make_ctl(wnd, L"BUTTON", L"涂黑测试", 0, IDC_B_TEST);
        make_ctl(wnd, L"BUTTON", L"添加文件", 0, IDC_B_ADD);
        make_ctl(wnd, L"BUTTON", L"移除选中", 0, IDC_B_REM);
        make_ctl(wnd, L"BUTTON", L"清空", 0, IDC_B_CLEAR);
        make_ctl(wnd, L"BUTTON", L"开始提取", 0, IDC_B_START);
        make_ctl(wnd, L"BUTTON", L"停止", 0, IDC_B_STOP);
        EnableWindow(GetDlgItem(wnd, IDC_B_STOP), FALSE);

        g_list = make_ctl(wnd, WC_LISTVIEWW, L"",
                          WS_BORDER | LVS_REPORT | LVS_SHOWSELALWAYS, IDC_LIST);
        ListView_SetExtendedListViewStyle(g_list, LVS_EX_FULLROWSELECT);
        LVCOLUMNW col = {0};
        col.mask = LVCF_TEXT | LVCF_WIDTH;
        col.pszText = (LPWSTR)L"文件";
        col.cx = 440;
        ListView_InsertColumn(g_list, 0, &col);
        col.pszText = (LPWSTR)L"状态";
        col.cx = 230;
        ListView_InsertColumn(g_list, 1, &col);
        col.pszText = (LPWSTR)L"清晰帧";
        col.cx = 70;
        ListView_InsertColumn(g_list, 2, &col);

        g_log = make_ctl(wnd, L"EDIT", L"",
                         WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY |
                         ES_AUTOVSCROLL, IDC_LOG);
        SendMessageA(g_log, EM_SETLIMITTEXT, 0, 0);

        g_pbar = make_ctl(wnd, PROGRESS_CLASSW, L"", PBS_SMOOTH, IDC_PBAR);

        DragAcceptFiles(wnd, TRUE);
        return 0;
    }
    case WM_SIZE:
        layout(wnd);
        return 0;
    case WM_DROPFILES: {
        HDROP drop = (HDROP)wp;
        int n = DragQueryFileW(drop, 0xFFFFFFFF, NULL, 0);
        wchar_t path[MAX_PATH * 2];
        for (int i = 0; i < n; i++) {
            DragQueryFileW(drop, (UINT)i, path, MAX_PATH * 2);
            list_add(path);
        }
        DragFinish(drop);
        return 0;
    }
    case WM_NOTIFY: {
        NMHDR *nm = (NMHDR *)lp;
        if (nm->idFrom == IDC_LIST && nm->code == (UINT)NM_DBLCLK) {
            NMITEMACTIVATE *ia = (NMITEMACTIVATE *)lp;
            if (ia->iItem >= 0 && ia->iItem < g_npaths) start_preview(ia->iItem);
        }
        return 0;
    }
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_B_ADD: add_files_dialog(wnd); break;
        case IDC_B_REM: if (!g_running) remove_selected(); break;
        case IDC_B_CLEAR:
            if (!g_running) { ListView_DeleteAllItems(g_list); g_npaths = 0; }
            break;
        case IDC_B_BROWSE: browse_outdir(wnd); break;
        case IDC_B_TEST: start_mask_test(); break;
        case IDC_B_HELP:
            MessageBoxW(wnd,
                L"两种选帧模式：默认按\u201c抽帧间隔\u201d分组，每组末尾\u201c锐度窗口\u201d\n"
                L"帧里挑最清晰的一张出图，胜者分数低于\u201c最低清晰度\u201d则整组\n"
                L"丢弃；把\u201c变化阈值\u201d填成大于 0 的数即切换为动态间隔模式。\n"
                L"双目文件两轨共用 cam0 测得的选帧布点：cam0/cam1 输出帧号\n"
                L"严格一致、逐号配对（轨1 跳过评分，不生成 cam1 的 csv）。\n\n"
                L"【抽帧间隔】（默认 8）\n"
                L"每 N 个源帧输出 1 张，出图密度 ≈ 帧率÷N（25fps、N=8 → 约3张/秒）。\n"
                L"想更密调小、更稀调大，与清晰度无关。\n"
                L"动态间隔模式下此参数不参与。\n\n"
                L"【锐度窗口】（留空 = 间隔÷2，填 0 = 不挑）\n"
                L"每组只有末尾 N 帧参与评分，输出其中分数最高的一张。\n"
                L"窗口越大出图越清晰，但时间戳抖动越大（最多偏 N-1 帧）。\n"
                L"建议范围 1~间隔；窗口=间隔时全组参评；填 0 固定取每组第 1 帧。\n"
                L"动态间隔模式下此参数不参与。\n\n"
                L"【变化阈值】（默认 0 = 固定间隔；填大于 0 的数启用动态间隔）\n"
                L"每帧与上一张出图帧比较平均灰度差（MAD），变化攒够阈值才\n"
                L"结算出图，期间仍挑最清晰的一张，最低清晰度照常过滤。\n"
                L"变化快\u2192出图密，静止\u2192自动变稀，相邻照片的画面变化量大致\n"
                L"恒定，给 SfM 建模喂图最合适。间隔自动在 4~32 帧间伸缩\n"
                L"（25fps \u2248 每秒 0.8~6 张），此模式下\u201c抽帧间隔/锐度窗口\u201d\n"
                L"被忽略。\n"
                L"定值方法：先用 0 跑一遍，看输出目录旁 sharpness_camN.csv 的\n"
                L"diff 列——静止段数值小、走动段数值大，把阈值定在两者之间；\n"
                L"调大出图更稀，调小更密。实测 8K 行走素材：8 \u2248 默认密度\n"
                L"（约 3 张/秒），12 约减半，常用 5~12。\n\n"
                L"【最低清晰度】（默认 0 = 不过滤）\n"
                L"清晰度分 = 缩略图上拉普拉斯响应的方差：边缘细节多分数高，\n"
                L"运动模糊/失焦分数低。用于丢掉模糊照片。\n"
                L"定值方法：先用 0 跑一遍，打开输出目录旁的 sharpness_cam0.csv\n"
                L"看分数分布（日志也打印 score min/avg/max），把阈值定在模糊段\n"
                L"与清晰段之间，通常先试 min~avg 之间的值，再按丢弃比例微调。\n"
                L"注意：数值与评分缩略图尺寸(-b)和场景内容相关，改参数要重标。\n\n"
                L"【JPEG质量】（默认 2）ffmpeg -q:v，越小越清晰，2 接近视觉无损。\n\n"
                L"【涂黑%】（默认 95，0 = 不涂黑；仅双目 OSV 文件生效）\n"
                L"抽帧编码时同步把每张图中心圆外的像素涂黑：圆心=画面\n"
                L"中心，保留半径 = 百分数×短边÷2。用于切掉鱼眼黑边和紧\n"
                L"贴黑边的失焦模糊环；涂黑区不产生图像梯度，SfM 特征点\n"
                L"天然落不进去，等价于给建模链喂 mask。\n"
                L"数字越小切得越多，保留面积 ≈ 百分数的平方\n"
                L"（95%→90%，88%→77%）。普通单流视频（普通 mp4 等）\n"
                L"没有鱼眼圆外黑边，自动跳过涂黑。\n"
                L"参考：DJI Osmo 360 样帧实测清晰边界约 93%，默认 95 是\n"
                L"用户在对比图上选定的值；拿不准就点「涂黑测试」看对比\n"
                L"图再定。涂黑并入编码滤镜，几乎不增加耗时，无二次损失。\n\n"
                L"【输出目录】留空 = 在各视频旁建 <视频名>_sharp\\。\n"
                L"双目文件（OSV）分 cam0、cam1 子目录；单路视频直接放入。",
                L"参数说明", MB_OK | MB_ICONINFORMATION);
            break;
        case IDC_B_START: start_batch(); break;
        case IDC_B_STOP: {
            g_cancel = 1;
            HANDLE child = (HANDLE)InterlockedExchangePointer(
                (void *volatile *)&g_cur_child, NULL);
            if (child) TerminateProcess(child, 1);
            gui_log(NULL, "[user] 停止请求，正在中止…\n");
            break;
        }
        }
        return 0;
    case APP_LOG:
        append_log((const char *)wp);
        free((void *)wp);
        return 0;
    case APP_STATUS: {
        StatusMsg *m = (StatusMsg *)wp;
        wchar_t text[128];
        if (m->phase == 0)
            wcscpy(text, L"探测解码器…");
        else if (m->phase == 1)
            _snwprintf(text, 128, L"cam%d 评分中 %lld 帧", m->track, m->a);
        else if (m->phase == 2)
            _snwprintf(text, 128, L"cam%d 输出中 %lld 张", m->track, m->a);
        else {
            _snwprintf(text, 128, L"cam%d 已选 %lld 张", m->track, m->a);
            if (m->idx != g_cnt_idx) { g_cnt_idx = m->idx; g_cnt_total = 0; }
            g_cnt_total += m->a;
            wchar_t cnt[32];
            _snwprintf(cnt, 32, L"%lld 张", g_cnt_total);
            ListView_SetItemText(g_list, m->idx, 2, cnt);
        }
        ListView_SetItemText(g_list, m->idx, 1, text);
        free(m);
        return 0;
    }
    case APP_FDONE: {
        DoneMsg *m = (DoneMsg *)wp;
        g_files_done++;
        wchar_t text[128], cnt[64] = L"";
        ListView_GetItemText(g_list, m->idx, 2, cnt, 64);
        if (m->rc == 0) _snwprintf(text, 128, L"完成 %ls", cnt);
        else if (m->rc == 2) wcscpy(text, L"已取消");
        else wcscpy(text, L"失败（见日志）");
        ListView_SetItemText(g_list, m->idx, 1, text);
        SendMessage(g_pbar, PBM_SETPOS,
                    (WPARAM)(g_files_done * 100 / (g_files_total ? g_files_total : 1)), 0);
        free(m);
        return 0;
    }
    case APP_TESTDONE:
        if (g_test_thread) { CloseHandle(g_test_thread); g_test_thread = NULL; }
        if (g_hwnd_test) InvalidateRect(g_hwnd_test, NULL, TRUE);
        return 0;
    case APP_ALLDONE:
        g_running = 0;
        EnableWindow(GetDlgItem(wnd, IDC_B_START), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_STOP), FALSE);
        EnableWindow(GetDlgItem(wnd, IDC_B_ADD), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_REM), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_CLEAR), TRUE);
        append_log("—— 批处理结束 ——\n");
        return 0;
    case WM_CTLCOLORSTATIC:
        /* 参数提示小字显示为灰色，与正常标签区分 */
        {
            int id = GetDlgCtrlID((HWND)lp);
            if (id >= IDC_HINT_SKIP && id <= IDC_HINT_DIFF) {
                static HBRUSH br = NULL;
                if (!br) br = GetSysColorBrush(COLOR_WINDOW);
                SetTextColor((HDC)wp, RGB(96, 96, 96));
                return (LRESULT)(INT_PTR)br;
            }
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(wnd, msg, wp, lp);
}

static int gui_run(void) {
    FreeConsole(); /* double-click launch: no empty console window behind */
    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_LISTVIEW_CLASSES | ICC_BAR_CLASSES };
    InitCommonControlsEx(&icc);
    WNDCLASSW wc = {0};
    wc.lpfnWndProc = wndproc;
    wc.hInstance = GetModuleHandleW(NULL);
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.lpszClassName = L"OsvSharpWnd";
    RegisterClassW(&wc);
    wc.lpfnWndProc = pv_wndproc;
    wc.lpszClassName = L"OsvSharpPreview";
    RegisterClassW(&wc);
    wc.lpfnWndProc = test_wndproc;
    wc.lpszClassName = L"OsvSharpTest";
    RegisterClassW(&wc);
    g_ffmpeg = find_ffmpeg(NULL);
    g_hwnd = CreateWindowW(L"OsvSharpWnd",
                           L"osvsharp — OSV 清晰帧批量提取（双击列表项预览）",
                           WS_OVERLAPPEDWINDOW,
                           CW_USEDEFAULT, CW_USEDEFAULT, 1120, 680,
                           NULL, NULL, wc.hInstance, NULL);
    ShowWindow(g_hwnd, SW_SHOW);
    UpdateWindow(g_hwnd);
    MSG msg;
    while (GetMessageW(&msg, NULL, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return (int)msg.wParam;
}

/* ------------------------------------------------------------------- main */

static void usage(void) {
    printf(
        "osvsharp - video/OSV in, sharp frames out (pure C + ffmpeg subprocess)\n\n"
        "usage: osvsharp <video1> [video2 ...] [options]\n"
        "  Drag & drop video files onto osvsharp.exe works: output goes to\n"
        "  <video dir>/<video name>_sharp (cam0/cam1 subdirs for two-stream\n"
        "  files like .OSV; attached-pic cover streams are ignored).\n"
        "  Two-stream files share one extraction plan measured on track 0:\n"
        "  cam0/cam1 output identical frame numbers (exact stereo pairing);\n"
        "  only cam0 gets a sharpness CSV.\n"
        "  GPS: a matching .SRT (DJI telemetry, per-frame) beside the video\n"
        "  is written into every jpeg as EXIF automatically; no SRT = no change.\n\n"
        "  -s, --skip <n>       抽帧间隔：每 n 个源帧输出 1 张（默认 8）\n"
        "  -k, --keep <n>       锐度窗口：每组末尾 n 帧里挑最清晰的一张\n"
        "                       （默认 skip/2；0 = 不挑，固定取每组第 1 帧）\n"
        "  -m, --min-score <f>  最低清晰度：胜者分数（拉普拉斯方差）低于\n"
        "                       此值则该组放弃出图（默认 0 = 不过滤；参考\n"
        "                       输出目录 sharpness_camN.csv 的分数分布定值）\n"
        "      --diff <f>        变化阈值：>0 启用动态间隔——当前帧与上一张\n"
        "                       出图帧的平均灰度差（MAD）>= f 才结算出图，\n"
        "                       窗内仍挑最清晰一张；变化快出图密、静止自动\n"
        "                       变稀（此模式下 -s/-k 被忽略）。实测 8K 行走\n"
        "                       素材 8 ≈ 默认密度、12 约减半，常用 5~12；\n"
        "                       定值参考 csv 的 diff 列分布\n"
        "      --min-gap <n>     动态模式最小出图间隔帧数（默认 4，\n"
        "                       25fps ≈ 最多 6 张/秒）\n"
        "      --max-gap <n>     动态模式最大出图间隔帧数，静止段兜底\n"
        "                       （默认 32）\n"
        "  -t, --track <n>      video track 0/1 (default: both)\n"
        "  -q, --jpeg-q <n>     JPEG质量，ffmpeg -q:v，越小越清晰（默认 2）\n"
        "  -b, --thumb <n>      评分缩略图边长，只影响打分与分数刻度（默认 512）\n"
        "      -mask <n>        边缘涂黑：抽帧编码时把每张图中心圆外涂黑，\n"
        "                       保留半径=n%%×短边÷2，越小切得越多\n"
        "                       （默认 95；0 = 不涂黑；仅双目 OSV 生效，\n"
        "                       普通单流视频自动跳过）\n"
        "  -o, --out <dir>      output directory (default: <video>_sharp)\n"
        "      --ffmpeg <path>  ffmpeg executable\n"
        "      --hwaccel <name> force cuda/vulkan/d3d11va/none\n"
        "      --nopause        do not wait for Enter at exit\n");
}

static void pause_if_interactive(int nopause) {
    if (nopause || !GetConsoleWindow()) return;
    printf("Press Enter to exit...\n");
    int ch;
    while ((ch = getchar()) != '\n' && ch != -1) {}
}

/* ffmpeg lookup: --ffmpeg flag > ffmpeg.exe beside osvsharp.exe > $FFMPEG > PATH */
static wchar_t *find_ffmpeg(const wchar_t *ffmpeg_arg) {
    if (ffmpeg_arg) return _wcsdup(ffmpeg_arg);
    wchar_t self[MAX_PATH];
    DWORD n = GetModuleFileNameW(NULL, self, MAX_PATH);
    if (n > 0 && n < MAX_PATH) {
        wchar_t *slash = wcsrchr(self, L'\\');
        if (slash) {
            size_t need = (size_t)(slash - self) + 16;
            wchar_t *cand = malloc(need * sizeof(wchar_t));
            _snwprintf(cand, need, L"%.*ls\\ffmpeg.exe", (int)(slash - self), self);
            DWORD attr = GetFileAttributesW(cand);
            if (attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY))
                return cand;
            free(cand);
        }
    }
    wchar_t *env = _wgetenv(L"FFMPEG");
    return _wcsdup(env ? env : L"ffmpeg");
}

/* "D:\\a\\b.MP4" -> dir="D:\\a" stem="b"；正斜杠路径（Git Bash/脚本
 * 传参）也认，否则 SRT 同名探测等按目录拼接的逻辑会拼错 */
static void split_stem(const wchar_t *path, wchar_t *dir, wchar_t *stem) {
    const wchar_t *slash = wcsrchr(path, L'\\');
    const wchar_t *fslash = wcsrchr(path, L'/');
    if (!slash || (fslash && fslash > slash)) slash = fslash;
    const wchar_t *base = slash ? slash + 1 : path;
    const wchar_t *dot = wcsrchr(base, L'.');
    size_t dirlen = slash ? (size_t)(slash - path) : 0;
    if (dirlen) { memcpy(dir, path, dirlen * sizeof(wchar_t)); dir[dirlen] = 0; }
    else wcscpy(dir, L".");
    if (dot) {
        wcsncpy(stem, base, (size_t)(dot - base));
        stem[dot - base] = 0;
    } else wcscpy(stem, base);
}

/* Enumerate real video streams via `ffmpeg -i` stderr (header parse only, no
 * decode).  Non-attached-pic video stream indices go into idx[], with each
 * stream's resolution in the parallel ws[]/hs[] (0 = unknown -- edge masking
 * is skipped for such tracks); *nattach counts attached-pic cover-art video
 * streams (DJI Avata360 MP4 carries one as 0:v:1 -- running the OSV
 * two-track logic on it yields an empty cam1).
 * Returns the count written to idx[], or -1 when nothing parseable came out
 * (caller then falls back to the old assume-two-tracks behaviour). */
static int probe_video_streams(const wchar_t *ffmpeg, const wchar_t *input,
                               int *idx, int *ws, int *hs, int maxn,
                               int *nattach, double *fps_out) {
    *nattach = 0;
    Cmd c = {0};
    cmd_add(&c, ffmpeg);
    cmd_add(&c, L"-nostdin");
    cmd_add(&c, L"-i");
    cmd_add(&c, input);
    Child ch;
    if (spawn(c.s, &ch, 2) != 0) { free(c.s); return -1; }
    free(c.s);

    size_t len = 0, cap = 65536;
    char *buf = malloc(cap);
    for (;;) {
        if (len == cap) { cap *= 2; buf = realloc(buf, cap); }
        DWORD got = 0;
        if (!ReadFile(ch.r, buf + len, (DWORD)(cap - len), &got, NULL) || got == 0)
            break;
        len += got;
    }
    buf[len < cap ? len : cap - 1] = 0;
    child_wait(&ch);

    int n = 0, nat = 0, parsed = 0;
    char *line = buf;
    double fpsv = 0.0;
    while (line) {
        char *nl = strchr(line, '\n');
        if (nl) *nl = 0;
        char *sp = strstr(line, "Stream #");
        if (sp && strstr(sp, "Video:")) {
            int si = -1;
            char *colon = strchr(sp + 8, ':');   /* "Stream #0:N[...]: ..." */
            if (colon && sscanf(colon + 1, "%d", &si) == 1 && si >= 0) {
                parsed = 1;
                if (strstr(sp, "attached pic")) {
                    nat++;
                } else if (n < maxn) {
                    /* 顺手抓 ", WxH" 分辨率（涂黑遮罩要用），同
                     * pv_probe_info 的解析方式 */
                    const char *q = sp;
                    idx[n] = si;
                    ws[n] = hs[n] = 0;
                    while (*q && *q != '\n') {
                        int wv, hv;
                        if (sscanf(q, ", %dx%d", &wv, &hv) == 2 && wv > 0) {
                            ws[n] = wv; hs[n] = hv;
                            break;
                        }
                        q++;
                    }
                    /* 顺手抓 ", 25 fps"（GPS 遥测按帧号→时间对齐 SRT 用）*/
                    if (fpsv <= 0.0) {
                        const char *fp = strstr(sp, "fps");
                        if (fp) {
                            const char *p = fp;
                            while (p > sp && p[-1] == ' ') p--;
                            const char *q2 = p;
                            while (q2 > sp && ((q2[-1] >= '0' && q2[-1] <= '9')
                                               || q2[-1] == '.'))
                                q2--;
                            if (p > q2) fpsv = strtod(q2, NULL);
                        }
                    }
                    n++;
                }
            }
        }
        line = nl ? nl + 1 : NULL;
    }
    free(buf);
    if (fps_out) *fps_out = fpsv;
    *nattach = nat;
    return parsed ? n : -1;
}

/* pass B 前清掉输出目录里上次运行留下的旧帧 jpg，防止换参数重跑
 * （输出变少）或上次中断时新旧帧混在一起。只删“纯数字帧号.jpg”
 * （本工具的命名格式），目录里用户自己的其他文件一律不动。 */
static int clean_stale_jpg(const wchar_t *dir) {
    wchar_t pat[MAX_PATH * 2];
    _snwprintf(pat, MAX_PATH * 2, L"%ls\\*.jpg", dir);
    WIN32_FIND_DATAW fd;
    HANDLE h = FindFirstFileW(pat, &fd);
    if (h == INVALID_HANDLE_VALUE) return 0;
    int n = 0;
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        const wchar_t *s = fd.cFileName;
        const wchar_t *dot = wcsrchr(s, L'.');
        if (!dot || dot == s || wcscmp(dot, L".jpg") != 0) continue;
        int alldigit = 1;
        for (const wchar_t *p = s; p < dot; p++)
            if (*p < L'0' || *p > L'9') { alldigit = 0; break; }
        if (!alldigit) continue;
        wchar_t full[MAX_PATH * 2];
        _snwprintf(full, MAX_PATH * 2, L"%ls\\%ls", dir, s);
        if (_wremove(full) == 0) n++;
    } while (FindNextFileW(h, &fd));
    FindClose(h);
    return n;
}

/* process one input end-to-end; 0 ok, 1 failed, 2 cancelled */
static int process_input(const wchar_t *ffmpeg, const wchar_t *input,
                         const wchar_t *outdir, int skip, int keep,
                         double min_score, double diff_th, int min_gap,
                         int max_gap, int track, int thumb, int jpeg_q,
                         int mask_pct, const wchar_t *hw_force, int file_idx) {
    g_file_idx = file_idx;
    Ctx cx = { ffmpeg, input, L"", thumb, jpeg_q, 0, 0 };
    {
        char *i8 = w_to_utf8(input), *o8 = w_to_utf8(outdir);
        if (diff_th > 0.0)
            LOG("=== %s ===\n  outdir: %s\n  mode: dynamic diff=%.1f gap=%d~%d"
                " min_score=%.1f q=%d mask=%d%%\n",
                i8, o8, diff_th, min_gap, max_gap, min_score, jpeg_q, mask_pct);
        else
            LOG("=== %s ===\n  outdir: %s\n  skip=%d keep=%d min_score=%.1f"
                " q=%d mask=%d%%\n",
                i8, o8, skip, keep, min_score, jpeg_q, mask_pct);
        free(i8); free(o8);
    }
    mkdirs_w(outdir);

    /* 只处理真实视频流：普通 MP4 可能带 attached-pic 封面流（如 DJI
     * Avata360 的 0:v:1），按 OSV 双目逻辑跑它只会得到空的 cam1。
     * 流列表解析失败时退回老行为（假定 0、1 两条轨）。 */
    int tracks[2] = { 0, 1 }, ntracks = 2;
    int tw[2] = { 0, 0 }, th[2] = { 0, 0 };   /* 每轨分辨率（涂黑遮罩用） */
    double fps = 0.0;   /* 首条真实视频流的帧率（GPS 遥测按时间对齐用） */
    {
    int vsidx[4], vsw[4], vsh[4], nattach = 0;
    int nreal = probe_video_streams(ffmpeg, input, vsidx, vsw, vsh, 4, &nattach,
                                    &fps);
        if (nreal > 0) {
            ntracks = nreal > 2 ? 2 : nreal;
            tracks[0] = vsidx[0];
            tracks[1] = ntracks > 1 ? vsidx[1] : -1;
            tw[0] = vsw[0]; th[0] = vsh[0];
            tw[1] = ntracks > 1 ? vsw[1] : 0;
            th[1] = ntracks > 1 ? vsh[1] : 0;
            if (nattach > 0)
                LOG("video streams: %d real, %d attached-pic cover ignored\n",
                    nreal, nattach);
        }
    }
    /* 双目判定以探测到的真实流数为准（-t 挑单轨后 ntracks 变 1，
     * 但文件仍是鱼眼双目，涂黑照常）；单流普通视频没有鱼眼圆外
     * 黑边，涂黑几何不适用，自动跳过。 */
    int dual = ntracks > 1;

    /* 大疆遥测：视频旁同名 .SRT（每帧 GPS/海拔）→ 出图时注入 EXIF。
     * 没有 SRT 行为不变。双目两路同一时刻共用同一条遥测。 */
    Telem tm = {0};
    tm.fps = fps;
    {
        wchar_t vdir[MAX_PATH], vstem[MAX_PATH], srtp[MAX_PATH * 2];
        split_stem(input, vdir, vstem);
        _snwprintf(srtp, MAX_PATH * 2, L"%ls\\%ls.srt", vdir, vstem);
        int ntm = srt_load(srtp, &tm);
        if (ntm > 0) {
            char *s8 = w_to_utf8(vstem);
            LOG("[gps] %s.srt: %d telemetry entries%s; writing EXIF GPS into jpegs\n",
                s8, ntm, fps > 0 ? "" : " (fps unknown, aligning by entry index)");
            free(s8);
        } else {
            LOG("[gps] no usable .SRT beside input, jpegs without EXIF GPS\n");
        }
    }
    if (track >= 0) {
        if (track < ntracks) {
            tracks[0] = tracks[track]; tw[0] = tw[track]; th[0] = th[track];
            ntracks = 1;
        }
        else {
            LOG("[track %d] not present, single-camera file -- skipped\n", track);
            ntracks = 0;
        }
    }

    ULONGLONG t0 = GetTickCount64();
    int rc = 0;
    /* 双轨：rig 两路看到的是同一场运动，选帧布点只在轨0 测量、两轨
     * 共享——cam0/cam1 输出相同帧号才能逐号配对（同号即同一时刻）。
     * 轨1 跳过评分，还省一遍解码；固定/动态两种模式都如此。 */
    Cand *plan = NULL;
    size_t nplan = 0;
    int shared = 0;
    for (int ti = 0; ti < ntracks && rc == 0; ti++) {
        int tr = tracks[ti];
        cx.vw = tw[ti]; cx.vh = th[ti];
        int multi = ntracks > 1;
        size_t dlen = wcslen(outdir) + 16;
        wchar_t *dir = malloc(dlen * sizeof(wchar_t));
        wchar_t camid[2] = { (wchar_t)(L'0' + ti), 0 };
        _snwprintf(dir, dlen, L"%ls%s%s", outdir, multi ? L"\\cam" : L"",
                   multi ? camid : L"");

        wchar_t *hw = NULL;
        if (hw_force) {
            cx.hw = wcscmp(hw_force, L"none") ? hw_force : L"";
        } else {
            STATUS(file_idx, 0, ti, 0);
            hw = probe_hwaccel(&cx, tr, 1); /* printing handled below */
            if (!hw) {
                if (ti == 0) {
                    LOG("no decodable video track %d in this file\n", tr);
                    free(dir);
                    return 1;
                }
                LOG("[track %d] not present, single-camera file -- skipped\n", ti);
                free(dir);
                break;
            }
            cx.hw = hw;
        }
        if (ti == 0) {
            char *h8 = w_to_utf8(cx.hw);
            LOG("hwaccel: %s%s\n", strlen(h8) ? h8 : "software",
                hw_force ? " (forced)" : "");
            free(h8);
        }
        Select sel = {0};
        long long decoded = 0;
        if (shared) {
            LOG("[track %d] pass A skipped: reusing track 0's extraction plan"
                " (%zu frames, stays paired with cam0)\n", ti, nplan);
        } else {
            LOG("[track %d] pass A: scoring frames...\n", ti);
            sel_init(&sel, skip, keep, min_score, diff_th, min_gap, max_gap, thumb);
            decoded = pass_a(&cx, tr, &sel);
            if (decoded == -2) { rc = 2; sel_free(&sel); free(dir); if (hw) free(hw); break; }
            if (decoded < 0) { rc = 1; sel_free(&sel); free(dir); if (hw) free(hw); break; }
        }

        if (!shared) {
            double mn = 1e30, mx = -1e30, sum = 0;
            double dmn = 1e30, dmx = -1e30, dsum = 0;
            for (size_t k = 0; k < sel.nall; k++) {
                double v = sel.all[k].score;
                if (v < mn) mn = v;
                if (v > mx) mx = v;
                sum += v;
                double dv = sel.all[k].diff;
                if (dv < dmn) dmn = dv;
                if (dv > dmx) dmx = dv;
                dsum += dv;
            }
            double avg = sel.nall ? sum / (double)sel.nall : 0.0;
            double davg = sel.nall ? dsum / (double)sel.nall : 0.0;
            if (!sel.nall) { mn = avg = mx = 0.0; dmn = davg = dmx = 0.0; }
            LOG("[track %d] decoded %lld frames, %zu candidates, %zu winners\n"
                "          score min=%.1f avg=%.1f max=%.1f\n"
                "          diff  min=%.1f avg=%.1f max=%.1f%s\n",
                ti, decoded, sel.nall, sel.nwin, mn, avg, mx, dmn, davg, dmx,
                diff_th > 0.0 ? "  (dynamic: diff vs last winner)" : "");

            wchar_t csvp[MAX_PATH * 2];
            _snwprintf(csvp, MAX_PATH * 2, L"%ls\\sharpness_cam%d.csv", outdir, ti);
            FILE *csv = _wfopen(csvp, L"wb");
            if (csv) {
                fprintf(csv, "frame,score,diff,chosen\n");
                for (size_t k = 0; k < sel.nall; k++)
                    fprintf(csv, "%lld,%.1f,%.1f,%d\n", sel.all[k].idx,
                            sel.all[k].score, sel.all[k].diff, sel.chosen[k]);
                fclose(csv);
            }
            /* 双轨：把轨0 的胜者表移出 sel 作为共享布点（轨1 直接用）*/
            if (ntracks > 1) {
                plan = sel.win;
                nplan = sel.nwin;
                sel.win = NULL;
                sel.nwin = sel.capwin = 0;
                shared = 1;
            }
        }

        mkdirs_w(dir);
        {
            int cleaned = clean_stale_jpg(dir);
            if (cleaned)
                LOG("[track %d] cleaned %d stale jpg(s) left by a previous run\n",
                    ti, cleaned);
        }
        Cand *wlist = shared ? plan : sel.win;
        size_t nw = shared ? nplan : sel.nwin;
        STATUS(file_idx, 3, ti, (long long)nw);
        wchar_t *maskp = NULL;
        if (mask_pct > 0 && !dual) {
            LOG("[track %d] pass B: writing %zu jpegs (edge mask skipped: single-stream video)...\n",
                ti, nw);
        } else if (mask_pct > 0 && cx.vw > 0 && cx.vh > 0) {
            maskp = make_mask_png(cx.vw, cx.vh, mask_pct, ti);
            if (maskp)
                LOG("[track %d] pass B: writing %zu jpegs (edge mask %d%%, in-filter single encode)...\n",
                    ti, nw, mask_pct);
            else
                LOG("[track %d] pass B: writing %zu jpegs (edge mask: cannot write temp png, skipped)...\n",
                    ti, nw);
        } else if (mask_pct > 0) {
            LOG("[track %d] pass B: writing %zu jpegs (edge mask skipped: resolution unknown)...\n",
                ti, nw);
        } else {
            LOG("[track %d] pass B: writing %zu jpegs...\n", ti, nw);
        }
        int pb = pass_b(&cx, tr, wlist, nw, dir, maskp, &tm);
        if (maskp) { _wunlink(maskp); free(maskp); }
        if (pb == -2) { rc = 2; sel_free(&sel); free(dir); if (hw) free(hw); break; }
        if (pb != 0) rc = 1;

        sel_free(&sel); free(dir);
        if (hw) free(hw);
    }
    free(plan);
    telem_free(&tm);
    LOG("finished in %.1f s\n", (GetTickCount64() - t0) / 1000.0);
    return rc;
}

int wmain(int argc, wchar_t **argv) {
    if (argc <= 1) return gui_run();  /* double-click: GUI; with args: CLI */
    SetConsoleOutputCP(CP_UTF8);

    /* --masktest <video> [out.png]：无界面自检，走与 GUI「涂黑测试」完全
     * 相同的代码路径（第一帧 × 85~99% 对比大图），把结果写成 PNG。 */
    if (argc >= 2 && !wcscmp(argv[1], L"--masktest")) {
        if (argc < 3) { printf("usage: --masktest <video> [out.png]\n"); return 2; }
        HANDLE th = CreateThread(NULL, 0, test_worker, _wcsdup(argv[2]), 0, NULL);
        WaitForSingleObject(th, INFINITE);
        CloseHandle(th);
        if (!g_test.sheet) { printf("mask test failed\n"); return 1; }
        size_t n = (size_t)g_test.w * g_test.h;
        unsigned char *rgb = malloc(n * 3);
        for (size_t i = 0; i < n; i++) {
            rgb[i * 3 + 0] = g_test.sheet[i * 3 + 2];
            rgb[i * 3 + 1] = g_test.sheet[i * 3 + 1];
            rgb[i * 3 + 2] = g_test.sheet[i * 3 + 0];
        }
        char *o8 = w_to_utf8(argc > 3 ? argv[3] : L"mask_test_sheet.png");
        int ok = o8 && stbi_write_png(o8, g_test.w, g_test.h, 3, rgb, g_test.w * 3);
        printf("%s: %dx%d sheet\n", ok ? "written" : "write failed", g_test.w, g_test.h);
        free(rgb);
        free(o8);
        return ok ? 0 : 1;
    }

    const wchar_t **inputs = malloc(sizeof(wchar_t *) * (size_t)argc);
    int ninputs = 0;
    const wchar_t *outdir_arg = NULL, *ffmpeg_arg = NULL, *hw_force = NULL;
    int skip = DEF_SKIP, keep = -1, track = -1, thumb = DEF_THUMB, jpeg_q = DEF_JPEG_Q;
    int mask_pct = DEF_MASK;
    double min_score = 0.0, diff_th = 0.0;
    int min_gap = DEF_ADAPT_MIN_GAP, max_gap = DEF_ADAPT_MAX_GAP;
    int nopause = 0;

    for (int i = 1; i < argc; i++) {
        const wchar_t *a = argv[i];
        if (!wcscmp(a, L"-h") || !wcscmp(a, L"--help")) { usage(); return 0; }
        else if (!wcscmp(a, L"-s") || !wcscmp(a, L"--skip")) skip = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-k") || !wcscmp(a, L"--keep")) keep = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-m") || !wcscmp(a, L"--min-score")) min_score = _wtof(argv[++i]);
        else if (!wcscmp(a, L"--diff")) diff_th = _wtof(argv[++i]);
        else if (!wcscmp(a, L"--min-gap")) min_gap = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"--max-gap")) max_gap = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-t") || !wcscmp(a, L"--track")) track = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-q") || !wcscmp(a, L"--jpeg-q")) jpeg_q = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-b") || !wcscmp(a, L"--thumb")) thumb = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-mask") || !wcscmp(a, L"--mask")) mask_pct = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-o") || !wcscmp(a, L"--out")) outdir_arg = argv[++i];
        else if (!wcscmp(a, L"--ffmpeg")) ffmpeg_arg = argv[++i];
        else if (!wcscmp(a, L"--hwaccel")) hw_force = argv[++i];
        else if (!wcscmp(a, L"--nopause")) nopause = 1;
        else inputs[ninputs++] = a;
    }
    if (ninputs == 0) { usage(); pause_if_interactive(nopause); return 1; }
    if (skip < 1) skip = 1;
    if (keep < 0) keep = (int)(0.5 * skip + 0.5);
    if (thumb < 32) thumb = 32;
    if (mask_pct < 0 || mask_pct > 100) mask_pct = DEF_MASK;
    if (diff_th < 0.0) diff_th = 0.0;
    if (min_gap < 1) min_gap = 1;
    if (max_gap < min_gap) max_gap = min_gap;

    wchar_t *ffmpeg = find_ffmpeg(ffmpeg_arg);

    /* a clear message beats five failed probes when ffmpeg is missing */
    {
        Cmd c = {0};
        cmd_add(&c, ffmpeg);
        cmd_add(&c, L"-version");
        if (run_quiet(c.s) != 0) {
            char *f8 = w_to_utf8(ffmpeg);
            printf("ERROR: cannot run ffmpeg (%s).\n"
                   "Put ffmpeg.exe next to osvsharp.exe, set FFMPEG, or add it "
                   "to PATH.\n", f8);
            free(f8);
            free(c.s);
            pause_if_interactive(nopause);
            return 1;
        }
        free(c.s);
    }

    int failed = 0;
    for (int i = 0; i < ninputs; i++) {
        wchar_t dir[MAX_PATH * 2], stem[MAX_PATH];
        wchar_t *outdir;
        if (outdir_arg) {
            if (ninputs == 1) {
                outdir = _wcsdup(outdir_arg);
            } else {
                split_stem(inputs[i], dir, stem);
                size_t n = wcslen(outdir_arg) + wcslen(stem) + 4;
                outdir = malloc(n * sizeof(wchar_t));
                _snwprintf(outdir, n, L"%ls\\%ls", outdir_arg, stem);
            }
        } else {
            /* <video dir>\<video stem>_sharp */
            split_stem(inputs[i], dir, stem);
            size_t n = wcslen(dir) + wcslen(stem) + 16;
            outdir = malloc(n * sizeof(wchar_t));
            _snwprintf(outdir, n, L"%ls\\%ls_sharp", dir, stem);
        }
        if (process_input(ffmpeg, inputs[i], outdir, skip, keep, min_score,
                          diff_th, min_gap, max_gap,
                          track, thumb, jpeg_q, mask_pct, hw_force, i) != 0)
            failed++;
        free(outdir);
    }

    printf("\n%d input(s), %d failed\n", ninputs, failed);
    free(ffmpeg);
    free(inputs);
    pause_if_interactive(nopause);
    return failed ? 1 : 0;
}
