// main.c
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "motor.h"

#define LINEBUF_SIZE 256
#define STOPBUF_SIZE 16

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);
    printf("connected\n");
    fflush(stdout);

    char linebuf[LINEBUF_SIZE];
    char stopbuf[STOPBUF_SIZE];

    while (1) {
        int linepos = 0;
        int c;
        while (1) {
            c = getchar();
            if (c < 0) continue;    
            if (c == '\r') continue;
            if (c == '\n') break;   
            if (linepos < LINEBUF_SIZE - 1) {
                linebuf[linepos++] = (char)c;
            }
        }
        linebuf[linepos] = '\0';

        if (strstr(linebuf, "STOP")) {
            printf("EMERGENCY STOP\n");
            fflush(stdout);
            continue;
        }

        unsigned delay, pulses, report, motor_sel;
        int dir;
        if (sscanf(linebuf,
                   "{\"delay\":%u,\"pulses\":%u,\"dir\":%d,\"report\":%u,\"motor\":%u}",
                   &delay, &pulses, &dir, &report, &motor_sel) != 5) {
            printf("bad cmd: %s\n", linebuf);
            fflush(stdout);
            continue;
        }

        printf("pkt: delay=%u, pulses=%u, dir=%d, report=%u, motor=%u\n",
               delay, pulses, dir, report, motor_sel);
        fflush(stdout);

        const uint elev_pins[5] = {11,13,0,1,9};
        const uint az_pins[5]   = {17,22,0,1,27};
        const uint *pins = motor_sel ? az_pins : elev_pins;

        Stepper motor;
        stepper_init(&motor,
                     pins[0],
                     pins[1],
                     pins[2],
                     pins[3],
                     pins[4] 
        );
        motor.delay_us = delay;
        motor.dir      = (dir > 0 ? 1 : -1);

        int stopidx = 0;
        for (unsigned i = 1; i <= pulses; i++) {
            stepper_move(&motor);

            if ((i % report) == 0) {
                printf("STATUS %d\n", motor.position);
                fflush(stdout);
            }

            int c2 = getchar_timeout_us(0);
            if (c2 >= 0) {
                if (c2 == '\n') {
                    stopbuf[stopidx] = '\0';
                    if (strstr(stopbuf, "STOP")) {
                        printf("STATUS %d\n", motor.position);
                        printf("EMERGENCY STOP\n");
                        fflush(stdout);
                        stepper_close(&motor);
                        break;
                    }
                    stopidx = 0;
                } else if (c2 != '\r') {
                    if (stopidx < STOPBUF_SIZE - 1) {
                        stopbuf[stopidx++] = (char)c2;
                    }
                }
            }
        }

        if (motor.enable_pin) {
            if ((pulses % report) != 0) {
                printf("STATUS %d\n", motor.position);
                fflush(stdout);
            }
            stepper_close(&motor);
        }
    }

    return 0;
}

