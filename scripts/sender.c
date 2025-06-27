/*
 * @file sender.c
 * @brief Host-side sender for interleaved dual-axis control of Pico steppers,
 *        with optional infinite back-and-forth motion, individual axis support,
 *        and CSV logging of positional data with Unix timestamps.
 *
 * Parses status JSON of the form {"pos_az":<int>,"pos_el":<int>} and logs:
 *    unix_time,pos_az,pos_el
 */

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
#include <sys/select.h>
#include <signal.h>
#include <sys/ioctl.h>  // for tcflush

#define DEFAULT_DELAY    225
#define DEFAULT_DEG_E    0
#define DEFAULT_DEG_A    0
#define DEFAULT_REPORT   100
#define DEFAULT_DEVICE   "/dev/ttyACM0"
#define STEP_ANGLE       1.8f
#define MICROSTEP        1
#define GEAR_TEETH       113

static volatile sig_atomic_t stop_flag = 0;
static FILE *g_write_fp = NULL;
static FILE *csv_fp = NULL;

// Thread to monitor stdin for emergency STOP (e.g., any key)
static void *stop_thread(void *_) {
    fd_set rfds;
    while (!stop_flag) {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = {0, 100000};  // 100ms
        if (select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) > 0) {
            char b;
            if (read(STDIN_FILENO, &b, 1) > 0) {
                stop_flag = 1;
                fprintf(g_write_fp, "STOP\n");
                fflush(g_write_fp);
            }
        }
    }
    return NULL;
}

// Configure serial: 115200 8N1 raw mode
static int configure_serial(int fd) {
    struct termios t;
    if (tcgetattr(fd, &t) < 0) return -1;
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    t.c_iflag = t.c_oflag = t.c_lflag = 0;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 10; // 1s timeout
    return tcsetattr(fd, TCSANOW, &t);
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -t, --time USEC      pulse delay (μs)         [default %d]\n"
        "  -e, --degree_e DEG   elevation sweep (°)      [default %d]\n"
        "  -a, --degree_a DEG   azimuth sweep (°)        [default %d]\n"
        "  -r, --report N       status every N steps     [default %d]\n"
        "  -l, --loop           infinite back-and-forth  [default off]\n"
        "  -s, --serial DEV     serial device            [default %s]\n"
        "  -h, --help           show this help\n",
        prog, DEFAULT_DELAY, DEFAULT_DEG_E,
        DEFAULT_DEG_A, DEFAULT_REPORT, DEFAULT_DEVICE);
    exit(1);
}

// Convert degrees to microstep pulses
static int calc_pulses(int deg) {
    return (int)(MICROSTEP * GEAR_TEETH * abs(deg) / STEP_ANGLE);
}

// Ensure CSV has header if empty
static void ensure_csv_header(FILE *fp) {
    fseek(fp, 0, SEEK_END);
    if (ftell(fp) == 0) {
        fprintf(fp, "unix_time,pos_az,pos_el\n");
        fflush(fp);
    }
}

// Send combined JSON command and read interleaved status reports
// Parses {"pos_az":<int>,"pos_el":<int>}
static int do_move_both(int deg_a, int deg_e,
                        int delay, int report,
                        FILE *wfp, FILE *rfp) {
    int pulses_az = calc_pulses(deg_a);
    int pulses_el = calc_pulses(deg_e);
    int dir_az = (deg_a >= 0 ? 1 : -1);
    int dir_el = (deg_e >= 0 ? 1 : -1);
    int max_steps = pulses_az > pulses_el ? pulses_az : pulses_el;
    int expected = max_steps / report + 1;
    int seen = 0;

    // Send JSON command
    fprintf(wfp,
        "{\"delay\":%d,\"pulses_az\":%d,\"dir_az\":%d,"
        "\"pulses_el\":%d,\"dir_el\":%d,\"report\":%d}\n",
        delay, pulses_az, dir_az,
        pulses_el, dir_el, report);
    fflush(wfp);

    char buf[128];
    while (!stop_flag && seen < expected && fgets(buf, sizeof(buf), rfp)) {
        if (strstr(buf, "EMERGENCY STOP")) {
            stop_flag = 1;
            break;
        }
        int pos_az, pos_el;
        if (sscanf(buf, "{\"pos_az\":%d,\"pos_el\":%d}", &pos_az, &pos_el) == 2) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long us = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
            // Print to console
            printf("%lld, az=%d, el=%d\n", us, pos_az, pos_el);
            // Log to CSV
            if (csv_fp) {
                fprintf(csv_fp, "%lld,%d,%d\n", us, pos_az, pos_el);
                fflush(csv_fp);
            }
            seen++;
        }
    }
    return stop_flag;
}

int main(int argc, char **argv) {
    int delay = DEFAULT_DELAY;
    int deg_e = DEFAULT_DEG_E;
    int deg_a = DEFAULT_DEG_A;
    int report = DEFAULT_REPORT;
    int loop = 0;
    char *dev = DEFAULT_DEVICE;

    struct option opts[] = {
        {"time",     1, 0, 't'},
        {"degree_e", 1, 0, 'e'},
        {"degree_a", 1, 0, 'a'},
        {"report",   1, 0, 'r'},
        {"loop",     0, 0, 'l'},
        {"serial",   1, 0, 's'},
        {"help",     0, 0, 'h'},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "t:e:a:r:ls:h", opts, NULL)) != -1) {
        switch (c) {
            case 't': delay = atoi(optarg); break;
            case 'e': deg_e = atoi(optarg); break;
            case 'a': deg_a = atoi(optarg); break;
            case 'r': report = atoi(optarg); break;
            case 'l': loop = 1; break;
            case 's': dev = optarg; break;
            default: usage(argv[0]);
        }
    }

    // Open CSV for logging
    csv_fp = fopen("serial_log.csv", "a+");
    if (!csv_fp) {
        perror("fopen serial_log.csv");
        return 1;
    }
    ensure_csv_header(csv_fp);

    int sweep_a = deg_a;
    int sweep_e = deg_e;
    if (deg_a == 0 && deg_e == 0) {
        sweep_a = sweep_e = 360;
    }

    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 1; }
    if (configure_serial(fd) < 0) { perror("configure_serial"); return 1; }
    g_write_fp = fdopen(fd, "w"); setvbuf(g_write_fp, NULL, _IONBF, 0);
    int fdr = dup(fd);
    FILE *rfp = fdopen(fdr, "r");

    // Start emergency-stop monitor
    pthread_t st;
    pthread_create(&st, NULL, stop_thread, NULL);

    if (loop) {
        int dir_a = 1, dir_e = 1;
        while (!stop_flag) {
            tcflush(fd, TCIFLUSH);
            do_move_both(sweep_a * dir_a, sweep_e * dir_e,
                         delay, report, g_write_fp, rfp);
            if (stop_flag) break;
            sleep(3);
            dir_a = -dir_a;
            dir_e = -dir_e;
        }
    } else {
        tcflush(fd, TCIFLUSH);
        do_move_both(sweep_a, sweep_e, delay, report, g_write_fp, rfp);
    }

    fclose(g_write_fp);
    fclose(rfp);
    if (csv_fp) fclose(csv_fp);
    pthread_join(st, NULL);
    return stop_flag ? 1 : 0;
}

