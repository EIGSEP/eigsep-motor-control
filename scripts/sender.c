// sender.c
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>

#define DEFAULT_DELAY    225
#define DEFAULT_ELEV     0
#define DEFAULT_AZIM     0
#define DEFAULT_REPORT   100
#define DEFAULT_DEVICE  "/dev/ttyACM0"
#define STEP_ANGLE     1.8f
#define MICROSTEP      4
#define GEAR_TEETH    113

static FILE *g_write_fp = NULL;  // unbuffered writer to Pico

// Any user input (followed by Enter) triggers an emergency STOP
static void *input_thread(void *_) {
    char buf[128];
    if (fgets(buf, sizeof(buf), stdin)) {
        // send STOP JSON
        fprintf(g_write_fp, "[\"STOP\"]\n");
        fflush(g_write_fp);
        printf("[sender] Emergency STOP sent\n");
    }
    return NULL;
}

// Scan for highest-index log and its last count
static void scan_logs(int *out_idx, int *out_off) {
    DIR *d = opendir(".");
    struct dirent *e;
    int max_idx = -1;
    char last_file[128] = {0};
    while ((e = readdir(d))) {
        int idx;
        if      (strcmp(e->d_name, "step_log.txt") == 0) idx = 0;
        else if (sscanf(e->d_name, "step_log_%d.txt", &idx) == 1) {}
        else continue;
        if (idx > max_idx) {
            max_idx = idx;
            strncpy(last_file, e->d_name, sizeof(last_file)-1);
        }
    }
    closedir(d);
    if (max_idx < 0) {
        *out_idx = 0;
        *out_off = 0;
        return;
    }
    FILE *f = fopen(last_file, "r");
    int last = 0;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char *c = strchr(line, ',');
        if (c) last = atoi(c + 1);
    }
    fclose(f);
    *out_idx = max_idx;
    *out_off = last;
}

// Configure serial port to raw 115200 8N1
static int configure_serial(int fd) {
    struct termios t;
    if (tcgetattr(fd, &t) < 0) { perror("tcgetattr"); return -1; }
    cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    t.c_iflag = t.c_oflag = t.c_lflag = 0;
    t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) { perror("tcsetattr"); return -1; }
    return 0;
}

static void usage(const char *p) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -t,--time USEC       pulse delay (µs)         [default %d]\n"
        "  -e,--degree_e DEG    box (elevation)°         [default %d]\n"
        "  -a,--degree_a DEG    antenna (azimuth)°       [default %d]\n"
        "  -r,--report N        report every N steps     [default %d]\n"
        "  -m,--max-size BYTES  rotate log at this size  [default no rotate]\n"
        "  -s,--serial DEV      serial port              [default %s]\n"
        "  -c,--stop            send STOP and exit\n"
        "  -h,--help            show this help\n",
        p,
        DEFAULT_DELAY, DEFAULT_ELEV,
        DEFAULT_AZIM,   DEFAULT_REPORT,
        DEFAULT_DEVICE
    );
    exit(1);
}

static int calc_pulses(int deg) {
    return (int)(MICROSTEP * GEAR_TEETH * abs(deg) / STEP_ANGLE);
}

typedef struct {
    FILE   *read_fp;
    FILE   *logf;
    int     offset;
    int     threshold;
    size_t  max_size;
    int     index;
} logger_args_t;

// Logger thread: read STATUS lines, timestamp & rotate logs as needed
static void *logger_thread(void *arg) {
    logger_args_t *la = arg;
    char line[128];
    while (fgets(line, sizeof(line), la->read_fp)) {
        if (strstr(line, "EMERGENCY STOP")) break;
        int cnt;
        if (sscanf(line, "STATUS %d", &cnt) == 1) {
            int adj = la->offset + cnt;
            // rotate if size exceeded
            if (la->max_size > 0) {
                struct stat st;
                if (fstat(fileno(la->logf), &st) == 0 &&
                    (size_t)st.st_size >= la->max_size) {
                    fclose(la->logf);
                    la->index++;
                    char fn[128];
                    snprintf(fn, sizeof(fn), "step_log_%d.txt", la->index);
                    la->logf = fopen(fn, "a+");
                }
            }
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long us = (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
            fprintf(la->logf, "%lld,%d\n", us, adj);
            fflush(la->logf);
            if (adj == la->threshold) break;
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    int    delay    = DEFAULT_DELAY;
    int    deg_e    = DEFAULT_ELEV;
    int    deg_a    = DEFAULT_AZIM;
    int    report   = DEFAULT_REPORT;
    size_t maxsz    = 0;
    char  *dev      = DEFAULT_DEVICE;
    int    send_stop= 0;

    struct option opts[] = {
        {"time",     required_argument, 0, 't'},
        {"degree_e", required_argument, 0, 'e'},
        {"degree_a", required_argument, 0, 'a'},
        {"report",   required_argument, 0, 'r'},
        {"max-size", required_argument, 0, 'm'},
        {"serial",   required_argument, 0, 's'},
        {"stop",     no_argument,       0, 'c'},
        {"help",     no_argument,       0, 'h'},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "t:e:a:r:m:s:ch", opts, NULL)) != -1) {
        switch (c) {
            case 't': delay    = atoi(optarg);      break;
            case 'e': deg_e    = atoi(optarg);      break;
            case 'a': deg_a    = atoi(optarg);      break;
            case 'r': report   = atoi(optarg);      break;
            case 'm': maxsz    = strtoul(optarg,0,10);break;
            case 's': dev      = optarg;            break;
            case 'c': send_stop= 1;                 break;
            default:  usage(argv[0]);
        }
    }

    // immediate STOP mode
    if (send_stop) {
        int fd = open(dev, O_RDWR|O_NOCTTY);
        if (fd < 0) { perror("open"); return 1; }
        configure_serial(fd);
        FILE *w = fdopen(fd, "w");
        setvbuf(w, NULL, _IONBF, 0);
        fprintf(w, "[\"STOP\"]\n");
        fclose(w);
        return 0;
    }

    // otherwise standard move + logging
    int idx, offset;
    scan_logs(&idx, &offset);

    int fdw = open(dev, O_RDWR|O_NOCTTY);
    if (fdw < 0) { perror("open"); return 1; }
    configure_serial(fdw);
    int fdr = dup(fdw);
    g_write_fp = fdopen(fdw, "w");
    setvbuf(g_write_fp, NULL, _IONBF, 0);
    FILE *read_fp = fdopen(fdr, "r");

    // launch interactive STOP thread
    pthread_t tid_in;
    pthread_create(&tid_in, NULL, input_thread, NULL);
    pthread_detach(tid_in);

    // prepare rotating log file
    char logn[128];
    if (maxsz > 0) {
        snprintf(logn, sizeof(logn), "step_log_%d.txt", idx);
        struct stat st;
        if (stat(logn, &st) == 0 && (size_t)st.st_size >= maxsz) {
            idx++;
            snprintf(logn, sizeof(logn), "step_log_%d.txt", idx);
        }
    } else {
        strcpy(logn, "step_log.txt");
    }
    FILE *logf = fopen(logn, "a+");

    int angle     = deg_a != 0 ? deg_a : deg_e;
    int pulses    = calc_pulses(angle);
    int dir       = angle >= 0 ? 1 : -1;
    int threshold = offset + pulses * dir;

    // start logger thread
    pthread_t tid_log;
    logger_args_t la = { read_fp, logf, offset, threshold, maxsz, idx };
    pthread_create(&tid_log, NULL, logger_thread, &la);

    // send the move command
    unsigned motor = deg_a != 0 ? 1U : 0U;
    fprintf(g_write_fp,
        "{\"delay\":%u,\"pulses\":%d,\"dir\":%d,\"report\":%u,\"motor\":%u}\n",
        delay, pulses, dir, report, motor);

    // wait for logging to finish
    pthread_join(tid_log, NULL);

    // cleanup
    fclose(g_write_fp);
    fclose(read_fp);
    if (logf) fclose(logf);
    return 0;
}

