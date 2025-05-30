#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor.h"

int main() {
    stdio_init_all();
    // wait for USB UART
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("connected\n"); fflush(stdout);

    // Initialize steppers
    const uint elev_pins[5] = {11, 13, 0, 1, 9};
    const uint az_pins[5]   = {17, 22, 0, 1, 27};
    Stepper elevation;
    Stepper azimuth;
    stepper_init(&elevation,
        elev_pins[0], elev_pins[1], elev_pins[2], elev_pins[3], elev_pins[4]);
    stepper_init(&azimuth,
        az_pins[0], az_pins[1], az_pins[2], az_pins[3], az_pins[4]);

    char buf[256];
    while (true) {
        if (!fgets(buf, sizeof(buf), stdin)) continue;
        // Handle immediate STOP command
        if (strstr(buf, "STOP")) {
            printf("EMERGENCY STOP\n"); fflush(stdout);
            continue;
        }
        // Parse JSON command
        unsigned delay_us, pulses, report, motor_sel;
        int dir;
        if (sscanf(buf,
            "{\"delay\":%u,\"pulses\":%u,\"dir\":%d,\"report\":%u,\"motor\":%u}",
            &delay_us, &pulses, &dir, &report, &motor_sel) != 5) {
            printf("bad cmd: %s", buf); fflush(stdout);
            continue;
        }
        // Choose motor
        Stepper *m = motor_sel ? &azimuth : &elevation;
        m->delay_us = delay_us;
        m->dir = (dir > 0 ? 1 : -1);

        // Stepping loop with STOP polling
        for (unsigned i = 1; i <= pulses; ++i) {
            stepper_move(m);
            if ((i % report) == 0) {
                printf("STATUS %ld,%ld\n",
                    (long)azimuth.position,
                    (long)elevation.position);
                fflush(stdout);
            }
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                // Received emergency STOP
                printf("EMERGENCY STOP\n"); fflush(stdout);
                break;
            }
        }
        // Final status
        printf("STATUS %ld,%ld\n",
            (long)azimuth.position,
            (long)elevation.position);
        fflush(stdout);
        stepper_close(m);
    }
    return 0;
}

