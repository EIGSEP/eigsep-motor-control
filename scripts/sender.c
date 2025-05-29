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
#include <signal.h>

#define DEFAULT_DELAY    225
#define DEFAULT_ELEV     0
#define DEFAULT_AZIM     0
#define DEFAULT_REPORT   100
#define DEFAULT_DEVICE  "/dev/ttyACM0"
#define STEP_ANGLE    1.8f
#define MICROSTEP     4
#define GEAR_TEETH  113

// Raw FD for async-signal-safe STOP
static int g_serial_fd = -1;

// Signal handler: send STOP over raw FD, then exit immediately
static void sigint_handler(int _) {
    if (g_serial_fd >= 0) {
        const char *stop = "[\"STOP\"]\n";
        write(g_serial_fd, stop, strlen(stop));
        fsync(g_serial_fd);
    }
    _exit(0);
}

// Scan existing log files to pick highest index and last count
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
    char buf[128];
    while (fgets(buf, sizeof(buf), f)) {
        char *c = strchr(buf, ',');
        if (c) last = atoi(c + 1);
    }
    fclose(f);

    *out_idx = max_idx;
    *out_off = last;
}

// Print usage and exit
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

// Convert degrees to required pulse count
static int calc_pulses(int deg) {
    return (int)(MICROSTEP * GEAR_TEETH * abs(deg) / STEP_ANGLE);
}

typedef struct {
    FILE    *read_fp;
    FILE    *logf;
    int      offset;
    int      threshold;
    size_t   max_size;
    int      index;
} logger_args_t;

// Logger thread: read STATUS lines, timestamp & write to rotating logs
static void *logger_thread(void *arg) {
    logger_args_t *la = arg;
    char line[128];
    while (fgets(line, sizeof(line), la->read_fp)) {
        if (strstr(line, "EMERGENCY STOP")) break;
        int cnt;
        if (sscanf(line, "STATUS %d", &cnt) == 1) {
            int adj = la->offset + cnt;

            // Rotate if file too large
            if (la->max_size > 0) {
                struct stat st;
                if (fstat(fileno(la->logf), &st)==0 &&
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

// Configure serial port: raw 115200 8N1
static int configure_serial(int fd) {
    struct termios t;
    if (tcgetattr(fd, &t) < 0) { perror("tcgetattr"); return -1; }
    cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    t.c_iflag = t.c_oflag = t.c_lflag = 0;
    t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &t) ? (perror("tcsetattr"), -1) : 0;
}

int main(int argc, char **argv) {
    signal(SIGINT, sigint_handler);

    int    delay    = DEFAULT_DELAY;
    int    deg_e    = DEFAULT_ELEV;
    int    deg_a    = DEFAULT_AZIM;
    int    report   = DEFAULT_REPORT;
    size_t max_size = 0;
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
            case 't': delay     = atoi(optarg);         break;
            case 'e': deg_e     = atoi(optarg);         break;
            case 'a': deg_a     = atoi(optarg);         break;
            case 'r': report    = atoi(optarg);         break;
            case 'm': max_size  = strtoul(optarg,0,10); break;
            case 's': dev       = optarg;               break;
            case 'c': send_stop = 1;                    break;
            default:  usage(argv[0]);
        }
    }

    // If only sending STOP, do so and exit immediately
    if (send_stop) {
        int fd = open(dev, O_RDWR | O_NOCTTY);
        if (fd < 0) { perror("open"); return 1; }
        g_serial_fd = fd;
        configure_serial(fd);
        const char *stop = "[\"STOP\"]\n";
        write(fd, stop, strlen(stop));
        fsync(fd);
        close(fd);
        return 0;
    }

    // Else perform a move + logging
    int idx, offset;
    scan_logs(&idx, &offset);

    int fd_w = open(dev, O_RDWR | O_NOCTTY);
    if (fd_w < 0) { perror("open"); return 1; }
    g_serial_fd = fd_w;
    if (configure_serial(fd_w) < 0) return 1;

    int fd_r = dup(fd_w);
    FILE *write_fp = fdopen(fd_w, "w");
    setvbuf(write_fp, NULL, _IONBF, 0);
    FILE *read_fp  = fdopen(fd_r, "r");

    // Prepare rotating log file
    char logname[128];
    if (max_size > 0) {
        snprintf(logname, sizeof(logname), "step_log_%d.txt", idx);
        struct stat st;
        if (stat(logname,&st)==0 && (size_t)st.st_size>=max_size) {
            idx++;
            snprintf(logname, sizeof(logname), "step_log_%d.txt", idx);
        }
    } else {
        strcpy(logname, "step_log.txt");
    }
    FILE *logf = fopen(logname, "a+");

    int angle     = deg_a != 0 ? deg_a : deg_e;
    int pulses    = calc_pulses(angle);
    int dir       = angle >= 0 ? 1 : -1;
    int threshold = offset + pulses * dir;

    pthread_t tid;
    logger_args_t la = {
        .read_fp   = read_fp,
        .logf      = logf,
        .offset    = offset,
        .threshold = threshold,
        .max_size  = max_size,
        .index     = idx
    };
    pthread_create(&tid, NULL, logger_thread, &la);

    unsigned motor = deg_a != 0 ? 1U : 0U;
    fprintf(write_fp,
        "{\"delay\":%u,\"pulses\":%d,\"dir\":%d,\"report\":%u,\"motor\":%u}\n",
        delay, pulses, dir, report, motor);
    fflush(write_fp);

    pthread_join(tid, NULL);

    fclose(write_fp);
    fclose(read_fp);
    if (logf) fclose(logf);
    return 0;
}

