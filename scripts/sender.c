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
#include <sys/select.h>

#define DEFAULT_DELAY    225
#define DEFAULT_DEG_E    0
#define DEFAULT_DEG_A    0
#define DEFAULT_REPORT   100
#define DEFAULT_DEVICE   "/dev/ttyACM0"
#define STEP_ANGLE       1.8f
#define MICROSTEP        4
#define GEAR_TEETH       113

static volatile sig_atomic_t stop_flag = 0;
static FILE *g_write_fp = NULL;

// Thread to monitor stdin for emergency STOP
static void *stop_thread(void *_) {
    fd_set rfds;
    while (!stop_flag) {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = {0, 100000}; // 100ms
        if (select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv) > 0) {
            char b;
            if (read(STDIN_FILENO, &b, 1) > 0) {
                stop_flag = 1;
                // forward STOP to Pico
                fprintf(g_write_fp, "[\"STOP\"]\n");
                fflush(g_write_fp);
            }
        }
    }
    return NULL;
}

static int configure_serial(int fd) {
    struct termios t;
    if (tcgetattr(fd, &t) < 0) return -1;
    cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    t.c_iflag = t.c_oflag = t.c_lflag = 0;
    t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &t);
}

static void usage(const char *p) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -t,--time USEC        pulse delay (µs)       [default %d]\n"
        "  -e,--degree_e DEG     elevation change (°)   [default %d]\n"
        "  -a,--degree_a DEG     azimuth change (°)     [default %d]\n"
        "  -r,--report N         report every N steps   [default %d]\n"
        "  -m,--max-size BYTES   rotate log at size     [default no rotate]\n"
        "  -s,--serial DEV       serial port            [default %s]\n"
        "  -c,--stop             send STOP and exit\n"
        "  -o,--observe          run full observe cycle\n"
        "  -h,--help             show this help\n",
        p, DEFAULT_DELAY, DEFAULT_DEG_E, DEFAULT_DEG_A,
        DEFAULT_REPORT, DEFAULT_DEVICE);
    exit(1);
}

static int calc_pulses(int deg) {
    return (int)(MICROSTEP * GEAR_TEETH * abs(deg) / STEP_ANGLE);
}

// scan combined logs for index and last offsets
static void scan_combined(int *idx, int *off_az, int *off_el) {
    DIR *d = opendir("."); struct dirent *e;
    int max = -1; char last[128] = {0};
    while ((e = readdir(d))) {
        int n;
        if (sscanf(e->d_name, "combined_step_log_%d.txt", &n) == 1) {
            if (n > max) { max = n; strcpy(last, e->d_name); }
        } else if (!strcmp(e->d_name, "combined_step_log.txt") && max < 0) {
            max = 0; strcpy(last, e->d_name);
        }
    }
    closedir(d);
    *idx = max < 0 ? 0 : max;
    *off_az = *off_el = 0;
    if (max >= 0) {
        FILE *f = fopen(last, "r"); char line[256];
        while (fgets(line, sizeof(line), f)) {
            long long ts; int az, el;
            if (sscanf(line, "%lld,%d,%d", &ts, &az, &el) == 3) {
                *off_az = az; *off_el = el;
            }
        }
        fclose(f);
    }
}

// perform move: send command then read fixed count of statuses
static int do_move(const char *motor, int deg, int delay,
                   int report, size_t maxsz,
                   int *idx, int *off_az, int *off_el,
                   FILE *wfp, FILE *rfp) {
    char fn[128];
    if (maxsz) {
        snprintf(fn, sizeof(fn), "combined_step_log_%d.txt", *idx);
        struct stat st;
        if (!stat(fn, &st) && (size_t)st.st_size >= maxsz) {
            ++*idx;
            snprintf(fn, sizeof(fn), "combined_step_log_%d.txt", *idx);
        }
    } else strcpy(fn, "combined_step_log.txt");
    FILE *lf = fopen(fn, "a+");

    int pulses = calc_pulses(deg);
    int dir = deg >= 0 ? 1 : -1;
    int id = strcmp(motor, "azimuth") == 0;
    // expected statuses = floor(pulses/report) + 1
    int expected = pulses / report + 1;
    int seen = 0;

    // dispatch command
    fprintf(wfp, "{\"delay\":%u,\"pulses\":%d,\"dir\":%d,\"report\":%u,\"motor\":%u}\n",
        delay, pulses, dir, report, (unsigned)id);
    fflush(wfp);

    char line[128];
    while (!stop_flag && fgets(line, sizeof(line), rfp)) {
        if (strstr(line, "EMERGENCY STOP")) {
            stop_flag = 1;
            break;
        }
        int az, el;
        if (sscanf(line, "STATUS %d,%d", &az, &el) == 2) {
            struct timeval tv; gettimeofday(&tv, NULL);
            long long us = (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
            fprintf(lf, "%lld,%d,%d\n", us, az, el);
            fflush(lf);
            if (++seen >= expected) break;
        }
    }
    // update offsets
    if (id) *off_az += pulses * dir;
    else     *off_el += pulses * dir;

    fclose(lf);
    return stop_flag;
}

int main(int argc, char **argv) {
    int delay = DEFAULT_DELAY;
    int deg_e = DEFAULT_DEG_E;
    int deg_a = DEFAULT_DEG_A;
    int report = DEFAULT_REPORT;
    size_t maxsz = 0;
    char *dev = DEFAULT_DEVICE;
    int send_stop = 0;
    int observe = 0;

    struct option opts[] = {
        {"time",     1, 0, 't'},
        {"degree_e", 1, 0, 'e'},
        {"degree_a", 1, 0, 'a'},
        {"report",   1, 0, 'r'},
        {"max-size", 1, 0, 'm'},
        {"serial",   1, 0, 's'},
        {"stop",     0, 0, 'c'},
        {"observe",  0, 0, 'o'},
        {"help",     0, 0, 'h'},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "t:e:a:r:m:s:coh", opts, NULL)) != -1) {
        switch (c) {
            case 't': delay = atoi(optarg); break;
            case 'e': deg_e = atoi(optarg); break;
            case 'a': deg_a = atoi(optarg); break;
            case 'r': report = atoi(optarg); break;
            case 'm': maxsz = strtoul(optarg, 0, 10); break;
            case 's': dev = optarg; break;
            case 'c': send_stop = 1; break;
            case 'o': observe = 1; break;
            default: usage(argv[0]);
        }
    }

    if (send_stop) {
        int fd = open(dev, O_RDWR|O_NOCTTY);
        configure_serial(fd);
        FILE *w = fdopen(fd, "w"); setvbuf(w, NULL, _IONBF, 0);
        fprintf(w, "[\"STOP\"]\n"); fclose(w);
        return 0;
    }

    int fdw = open(dev, O_RDWR|O_NOCTTY);
    configure_serial(fdw);
    int fdr = dup(fdw);
    g_write_fp = fdopen(fdw, "w"); setvbuf(g_write_fp, NULL, _IONBF, 0);
    FILE *rfp = fdopen(fdr, "r");

    // launch stop monitor
    pthread_t st;
    pthread_create(&st, NULL, stop_thread, NULL);

    int idx, off_az, off_el;
    scan_combined(&idx, &off_az, &off_el);

    if (observe) {
        int moved = 0, dir_e = 1;
        while (!stop_flag) {
            do_move("azimuth", 360, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
            if (stop_flag) break;
            moved += 10 * dir_e;
            do_move("elevation", 10 * dir_e, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
            if (stop_flag) break;
            do_move("azimuth", -360, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
            if (stop_flag) break;
            moved += 10 * dir_e;
            do_move("elevation", 10 * dir_e, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
            if (stop_flag) break;
            if (abs(moved) >= 360) { dir_e = -dir_e; moved = 0; }
        }
    } else {
        if (deg_a) do_move("azimuth", deg_a, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
        if (deg_e) do_move("elevation", deg_e, delay, report, maxsz, &idx, &off_az, &off_el, g_write_fp, rfp);
    }

    fclose(g_write_fp);
    fclose(rfp);
    return 0;
}

