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
 * The metric and the window arithmetic are ports of spirula-studio's
 * src/video/shaders/video.slang and src/app/FrameExtract.cpp, so results are
 * comparable with that tool's `spirula sam extract`.
 *
 * Build:  clang -std=c99 -O2 osvsharp.c -o osvsharp.exe
 * Usage:  osvsharp <input.OSV> [outdir] [-s skip] [-k keep] [-m min_score]
 *                     [-t track] [-q jpeg_q] [-b thumb] [--ffmpeg PATH]
 *                     [--hwaccel NAME] [--no-hwaccel]
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

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "comctl32.lib")

#define DEF_SKIP    8      /* 25 fps source → ~3 written frames per second */
#define DEF_JPEG_Q  2      /* ffmpeg mjpeg -q:v (lower = better)            */
#define DEF_THUMB   512    /* spirula's thumbnail size                      */

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

static int spawn(const wchar_t *cmdline, Child *c, int mute_stderr) {
    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, TRUE };
    HANDLE rh, wh;
    if (!CreatePipe(&rh, &wh, &sa, 0)) return -1;
    SetHandleInformation(wh, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT);

    STARTUPINFOW si;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = wh;
    si.hStdError = mute_stderr ? NULL : GetStdHandle(STD_ERROR_HANDLE);
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

/* ------------------------------------------- sliding-window frame selection */

typedef struct { long long idx; double score; } Cand;

typedef struct {
    int skip, keep;
    double min_score;
    Cand *win;  size_t nwin,  capwin;   /* chosen frames                */
    Cand *all;  size_t nall,  capall;   /* every candidate, for the csv */
    char  *chosen;                      /* parallel to all              */
    Cand *w; int wsize, wcap;           /* live window                  */
    long long fc;
} Select;

static void sel_init(Select *s, int skip, int keep, double min_score) {
    memset(s, 0, sizeof(*s));
    s->skip = skip;
    s->keep = keep;
    s->min_score = min_score;
    s->wcap = keep > 0 ? keep : 1;
    s->w = malloc(sizeof(Cand) * (size_t)s->wcap);
}

static void push_cand(Cand **v, size_t *n, size_t *cap, Cand c) {
    if (*n == *cap) {
        *cap = *cap ? *cap * 2 : 1024;
        *v = realloc(*v, sizeof(Cand) * *cap);
    }
    (*v)[(*n)++] = c;
}

/* Port of FrameExtract.cpp's decode loop: candidate test, window push,
 * and "write the sharpest of the window every `skip` source frames". */
static void sel_frame(Select *s, long long i, double score) {
    int cand = s->keep > 0
        ? (int)(((i + s->keep) % s->skip) < s->keep)
        : (int)((i % s->skip) == 0);
    if (!cand) { s->fc++; return; }

    push_cand(&s->all, &s->nall, &s->capall, (Cand){ i, score });
    s->chosen = realloc(s->chosen, s->nall);
    s->chosen[s->nall - 1] = 0;

    if (s->wsize == s->wcap) {
        memmove(s->w, s->w + 1, sizeof(Cand) * (size_t)(s->wcap - 1));
        s->wsize--;
    }
    s->w[s->wsize].idx = i;
    s->w[s->wsize].score = score;
    s->wsize++;

    if (s->keep > 0) s->fc++;
    int write_now = (s->fc % s->skip) == 0;
    if (s->keep == 0) s->fc++;
    if (!write_now) return;

    int best = 0;
    for (int k = 1; k < s->wsize; k++)
        if (s->w[k].score > s->w[best].score) best = k;
    /* mark chosen in `all` */
    for (size_t k = 0; k < s->nall; k++)
        if (s->all[k].idx == s->w[best].idx) { s->chosen[k] = 1; break; }
    if (s->w[best].score >= s->min_score)
        push_cand(&s->win, &s->nwin, &s->capwin, s->w[best]);
    s->wsize = 0;
}

/* ------------------------------------------------------------ pass runners */

typedef struct {
    const wchar_t *ffmpeg;   /* ffmpeg executable                */
    const wchar_t *input;    /* the .OSV path                    */
    const wchar_t *hw;       /* L"" = software                   */
    int thumb, jpeg_q;
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
        sel_frame(sel, i, laplacian_var(frame, cx->thumb));
        if (++i % 250 == 0) STATUS(g_file_idx, 1, track, i);
    }
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

/* write one winner jpeg; returns 0 ok */
static int write_jpeg(const wchar_t *dir, long long idx, const uint8_t *d, size_t n) {
    wchar_t path[MAX_PATH];
    _snwprintf(path, MAX_PATH, L"%ls\\%05lld.jpg", dir, idx);
    FILE *f = _wfopen(path, L"wb");
    if (!f) return -1;
    fwrite(d, 1, n, f);
    fclose(f);
    return 0;
}

/* pass B: re-decode, encode every frame as mjpeg over the pipe, and write
 * only the winners (counted by frame number -- no select expression, whose
 * length ffmpeg's expression parser cannot take for a few hundred frames). */
static int pass_b(const Ctx *cx, int track, const Cand *winners, size_t nwin,
                  const wchar_t *outdir) {
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
                    if (write_jpeg(outdir, n, buf + start, end - start) != 0) {
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

/* ------------------------------------------------------------------- GUI */

#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "shell32.lib")

#define APP_LOG      (WM_APP + 1)   /* wParam: malloc'd utf8 line */
#define APP_STATUS   (WM_APP + 2)   /* wParam: malloc'd StatusMsg */
#define APP_FDONE    (WM_APP + 3)   /* wParam: malloc'd {idx, rc} */
#define APP_ALLDONE  (WM_APP + 4)

#define IDC_LIST     1001
#define IDC_LOG      1002
#define IDC_E_SKIP   1011
#define IDC_E_KEEP   1012
#define IDC_E_MIN    1013
#define IDC_E_Q      1014
#define IDC_E_OUT    1015
#define IDC_B_ADD    1021
#define IDC_B_REM    1022
#define IDC_B_CLEAR  1023
#define IDC_B_START  1024
#define IDC_B_STOP   1025
#define IDC_B_BROWSE 1026
#define IDC_PBAR     1031

typedef struct { int idx, phase, track; long long a; } StatusMsg;
typedef struct { int idx, rc; } DoneMsg;

/* defined in the CLI/main section below */
static wchar_t *find_ffmpeg(const wchar_t *ffmpeg_arg);
static void split_stem(const wchar_t *path, wchar_t *dir, wchar_t *stem);
static int process_input(const wchar_t *ffmpeg, const wchar_t *input,
                         const wchar_t *outdir, int skip, int keep,
                         double min_score, int track, int thumb, int jpeg_q,
                         const wchar_t *hw_force, int file_idx);

static HWND g_hwnd, g_list, g_log, g_pbar;
static HFONT g_font;
static wchar_t (*g_paths)[MAX_PATH];
static int g_npaths, g_cappaths;
static int g_running, g_files_done, g_files_total;

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
    int len = GetWindowTextLengthA(g_log);
    SendMessageA(g_log, EM_SETSEL, len, len);
    SendMessageA(g_log, EM_REPLACESEL, FALSE, (LPARAM)utf8);
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

    int translated = 0;
    int skip = GetDlgItemInt(g_hwnd, IDC_E_SKIP, &translated, FALSE);
    if (!translated || skip < 1) skip = DEF_SKIP;
    int keep = GetDlgItemInt(g_hwnd, IDC_E_KEEP, &translated, FALSE);
    if (!translated || keep < 0) keep = (int)(0.5 * skip + 0.5); /* empty = auto */
    int q = GetDlgItemInt(g_hwnd, IDC_E_Q, &translated, FALSE);
    if (!translated || q < 1 || q > 31) q = DEF_JPEG_Q;
    double minscore = 0.0;
    {
        wchar_t buf[64];
        GetDlgItemTextW(g_hwnd, IDC_E_MIN, buf, 64);
        minscore = _wtof(buf);
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
                               -1, DEF_THUMB, q, NULL, i);
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
    MoveWindow(GetDlgItem(wnd, IDC_E_SKIP), x + 66, y, 40, 23, TRUE); x += 124;
    MoveWindow(GetDlgItem(wnd, IDC_E_KEEP), x + 66, y, 40, 23, TRUE); x += 124;
    MoveWindow(GetDlgItem(wnd, IDC_E_MIN), x + 78, y, 48, 23, TRUE); x += 142;
    MoveWindow(GetDlgItem(wnd, IDC_E_Q), x + 70, y, 34, 23, TRUE); x += 114;
    MoveWindow(GetDlgItem(wnd, IDC_E_OUT), x + 66, y, w - x - 88, 23, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_BROWSE), w - 78, y, 68, 23, TRUE);
    y += 34;
    MoveWindow(GetDlgItem(wnd, IDC_B_ADD), 10, y, 90, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_REM), 105, y, 90, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_CLEAR), 200, y, 60, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_START), w - 180, y, 80, 26, TRUE);
    MoveWindow(GetDlgItem(wnd, IDC_B_STOP), w - 90, y, 80, 26, TRUE);
    MoveWindow(g_pbar, 270, y + 4, w - 460, 18, TRUE);
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

static LRESULT CALLBACK wndproc(HWND wnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE: {
        g_font = CreateFontW(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                             DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0,
                             L"Microsoft YaHei UI");
        add_label(wnd, L"抽帧间隔", 10, 62);
        add_label(wnd, L"锐度窗口", 134, 62);
        add_label(wnd, L"最低清晰度", 258, 76);
        add_label(wnd, L"JPEG质量", 400, 66);
        add_label(wnd, L"输出目录", 514, 62);
        make_ctl(wnd, L"EDIT", L"8", WS_BORDER | ES_AUTOHSCROLL, IDC_E_SKIP);
        make_ctl(wnd, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL, IDC_E_KEEP);
        make_ctl(wnd, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL, IDC_E_MIN);
        make_ctl(wnd, L"EDIT", L"2", WS_BORDER | ES_AUTOHSCROLL, IDC_E_Q);
        make_ctl(wnd, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL, IDC_E_OUT);
        make_ctl(wnd, L"BUTTON", L"浏览…", 0, IDC_B_BROWSE);
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
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_B_ADD: add_files_dialog(wnd); break;
        case IDC_B_REM: if (!g_running) remove_selected(); break;
        case IDC_B_CLEAR:
            if (!g_running) { ListView_DeleteAllItems(g_list); g_npaths = 0; }
            break;
        case IDC_B_BROWSE: browse_outdir(wnd); break;
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
            ListView_SetItemText(g_list, m->idx, 2, (LPWSTR)(text + 8));
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
    case APP_ALLDONE:
        g_running = 0;
        EnableWindow(GetDlgItem(wnd, IDC_B_START), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_STOP), FALSE);
        EnableWindow(GetDlgItem(wnd, IDC_B_ADD), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_REM), TRUE);
        EnableWindow(GetDlgItem(wnd, IDC_B_CLEAR), TRUE);
        append_log("—— 批处理结束 ——\n");
        return 0;
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
    g_hwnd = CreateWindowW(wc.lpszClassName,
                           L"osvsharp — OSV 视频清晰帧批量提取",
                           WS_OVERLAPPEDWINDOW,
                           CW_USEDEFAULT, CW_USEDEFAULT, 1000, 680,
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
        "  <video dir>/<video name>_sharp\\cam0, cam1.\n\n"
        "  -s, --skip <n>       write one frame every n source frames (default 8)\n"
        "  -k, --keep <n>       sharpest-of-window size (default: skip/2)\n"
        "  -m, --min-score <f>  absolute Laplacian-variance floor (default 0)\n"
        "  -t, --track <n>      video track 0/1 (default: both)\n"
        "  -q, --jpeg-q <n>     mjpeg quality, ffmpeg -q:v (default 2)\n"
        "  -b, --thumb <n>      scoring thumbnail size (default 512)\n"
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

/* "D:\\a\\b.MP4" -> dir="D:\\a" stem="b" */
static void split_stem(const wchar_t *path, wchar_t *dir, wchar_t *stem) {
    const wchar_t *slash = wcsrchr(path, L'\\');
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

/* process one input end-to-end; 0 ok, 1 failed, 2 cancelled */
static int process_input(const wchar_t *ffmpeg, const wchar_t *input,
                         const wchar_t *outdir, int skip, int keep,
                         double min_score, int track, int thumb, int jpeg_q,
                         const wchar_t *hw_force, int file_idx) {
    g_file_idx = file_idx;
    Ctx cx = { ffmpeg, input, L"", thumb, jpeg_q };
    {
        char *i8 = w_to_utf8(input), *o8 = w_to_utf8(outdir);
        LOG("=== %s ===\n  outdir: %s\n  skip=%d keep=%d min_score=%.1f q=%d\n",
            i8, o8, skip, keep, min_score, jpeg_q);
        free(i8); free(o8);
    }
    mkdirs_w(outdir);

    int tracks[2] = { 0, 1 }, ntracks = 2;
    if (track >= 0) { tracks[0] = track; ntracks = 1; }

    ULONGLONG t0 = GetTickCount64();
    int rc = 0;
    for (int ti = 0; ti < ntracks && rc == 0; ti++) {
        int tr = tracks[ti];
        int multi = ntracks > 1;
        size_t dlen = wcslen(outdir) + 16;
        wchar_t *dir = malloc(dlen * sizeof(wchar_t));
        wchar_t camid[2] = { (wchar_t)(L'0' + tr), 0 };
        _snwprintf(dir, dlen, L"%ls%s%s", outdir, multi ? L"\\cam" : L"",
                   multi ? camid : L"");

        wchar_t *hw = NULL;
        if (hw_force) {
            cx.hw = wcscmp(hw_force, L"none") ? hw_force : L"";
        } else {
            STATUS(file_idx, 0, tr, 0);
            hw = probe_hwaccel(&cx, tr, 1); /* printing handled below */
            if (!hw) {
                if (tr == 0) {
                    LOG("no decodable video track 0 in this file\n");
                    free(dir);
                    return 1;
                }
                LOG("[track 1] not present, single-camera file -- skipped\n");
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
        LOG("[track %d] pass A: scoring frames...\n", tr);

        Select sel;
        sel_init(&sel, skip, keep, min_score);
        long long decoded = pass_a(&cx, tr, &sel);
        if (decoded == -2) { rc = 2; free(dir); if (hw) free(hw); break; }
        if (decoded < 0) { rc = 1; free(dir); if (hw) free(hw); break; }

        double mn = 1e30, mx = -1e30, sum = 0;
        for (size_t k = 0; k < sel.nall; k++) {
            double v = sel.all[k].score;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
        }
        double avg = sel.nall ? sum / (double)sel.nall : 0.0;
        LOG("[track %d] decoded %lld frames, %zu candidates, %zu winners\n"
            "          score min=%.1f avg=%.1f max=%.1f\n",
            tr, decoded, sel.nall, sel.nwin, mn, avg, mx);

        wchar_t csvp[MAX_PATH * 2];
        _snwprintf(csvp, MAX_PATH * 2, L"%ls\\sharpness_cam%d.csv", outdir, tr);
        FILE *csv = _wfopen(csvp, L"wb");
        if (csv) {
            fprintf(csv, "frame,score,chosen\n");
            for (size_t k = 0; k < sel.nall; k++)
                fprintf(csv, "%lld,%.1f,%d\n", sel.all[k].idx, sel.all[k].score,
                        sel.chosen[k]);
            fclose(csv);
        }

        mkdirs_w(dir);
        STATUS(file_idx, 3, tr, (long long)sel.nwin);
        LOG("[track %d] pass B: writing %zu jpegs...\n", tr, sel.nwin);
        int pb = pass_b(&cx, tr, sel.win, sel.nwin, dir);
        if (pb == -2) { rc = 2; free(sel.win); free(sel.all); free(sel.chosen); free(sel.w); free(dir); if (hw) free(hw); break; }
        if (pb != 0) rc = 1;

        free(sel.win); free(sel.all); free(sel.chosen); free(sel.w); free(dir);
        if (hw) free(hw);
    }
    LOG("finished in %.1f s\n", (GetTickCount64() - t0) / 1000.0);
    return rc;
}

int wmain(int argc, wchar_t **argv) {
    if (argc <= 1) return gui_run();  /* double-click: GUI; with args: CLI */
    SetConsoleOutputCP(CP_UTF8);

    const wchar_t **inputs = malloc(sizeof(wchar_t *) * (size_t)argc);
    int ninputs = 0;
    const wchar_t *outdir_arg = NULL, *ffmpeg_arg = NULL, *hw_force = NULL;
    int skip = DEF_SKIP, keep = -1, track = -1, thumb = DEF_THUMB, jpeg_q = DEF_JPEG_Q;
    double min_score = 0.0;
    int nopause = 0;

    for (int i = 1; i < argc; i++) {
        const wchar_t *a = argv[i];
        if (!wcscmp(a, L"-h") || !wcscmp(a, L"--help")) { usage(); return 0; }
        else if (!wcscmp(a, L"-s") || !wcscmp(a, L"--skip")) skip = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-k") || !wcscmp(a, L"--keep")) keep = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-m") || !wcscmp(a, L"--min-score")) min_score = _wtof(argv[++i]);
        else if (!wcscmp(a, L"-t") || !wcscmp(a, L"--track")) track = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-q") || !wcscmp(a, L"--jpeg-q")) jpeg_q = _wtoi(argv[++i]);
        else if (!wcscmp(a, L"-b") || !wcscmp(a, L"--thumb")) thumb = _wtoi(argv[++i]);
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
                          track, thumb, jpeg_q, hw_force, i) != 0)
            failed++;
        free(outdir);
    }

    printf("\n%d input(s), %d failed\n", ninputs, failed);
    free(ffmpeg);
    free(inputs);
    pause_if_interactive(nopause);
    return failed ? 1 : 0;
}
