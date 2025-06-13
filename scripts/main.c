/**
 * @file main.c
 * @brief Application entry point: USB serial command loop for stepper control.
 *
 * This program runs on a Raspberry Pi Pico, initializing two stepper motors
 * (elevation and azimuth). It waits for a USB serial connection, then listens
 * for JSON commands to move the motors by a specified number of pulses,
 * with user-defined step delay, direction, and status reporting frequency.
 * Supports an emergency STOP command to immediately halt motion.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor.h"

/**
 * @brief Main program loop.
 *
 * Initializes serial I/O, waits for USB UART connection, and configures two
 * Stepper instances for elevation and azimuth axes. Enters an infinite loop
 * reading lines from stdin, handling emergency STOP, parsing JSON commands,
 * executing step pulses with real-time STOP polling, and reporting positions.
 *
 * JSON command format:
 * @code
 * {"delay":<microseconds>,
 *  "pulses":<number_of_steps>,
 *  "dir":<1_or_-1>,
 *  "report":<status_interval_steps>,
 *  "motor":<0_for_elevation,_1_for_azimuth> }
 * @endcode
 *
 * @return Never returns under normal operation.
 */
int main() {
    /* Initialize all stdio, including USB serial */
    stdio_init_all();
    /* Wait until the host opens the USB serial port */
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("connected\n"); fflush(stdout);

    /* Configure pin arrays for elevation and azimuth steppers */
    const uint elev_pins[5] = {21, 18, 0, 1, 19};
    const uint az_pins[5]   = {11, 12, 0, 1, 10};
    Stepper elevation;
    Stepper azimuth;
    /* Initialize each Stepper struct with its GPIO configuration */
    stepper_init(&elevation,
        elev_pins[0], elev_pins[1], elev_pins[2], elev_pins[3], elev_pins[4]);
    stepper_init(&azimuth,
        az_pins[0], az_pins[1], az_pins[2], az_pins[3], az_pins[4]);

    char buf[256];
    /* Main command-processing loop */
    while (true) {
        /* Read a line from stdin (blocking) */
        if (!fgets(buf, sizeof(buf), stdin)) continue;

        /* Check for emergency STOP keyword */
        if (strstr(buf, "STOP")) {
            printf("EMERGENCY STOP\n"); fflush(stdout);
            continue;
        }

        /* Parse JSON command fields into variables */
        unsigned delay_us, pulses, report, motor_sel;
        int dir;
        if (sscanf(buf,
            "{\"delay\":%u,\"pulses\":%u,\"dir\":%d,\"report\":%u,\"motor\":%u}",
            &delay_us, &pulses, &dir, &report, &motor_sel) != 5) {
            printf("bad cmd: %s", buf); fflush(stdout);
            continue;
        }

        /* Select the appropriate motor and set parameters */
        Stepper *m = motor_sel ? &azimuth : &elevation;
        m->delay_us = delay_us;
        m->dir = (dir > 0 ? 1 : -1);

        /* Step loop with periodic status reporting and STOP polling */
        for (unsigned i = 1; i <= pulses; ++i) {
            stepper_move(m);
            if ((i % report) == 0) {
                printf("STATUS %ld,%ld\n",
                    (long)azimuth.position,
                    (long)elevation.position);
                fflush(stdout);
            }
            /* Non-blocking check for additional input (emergency STOP) */
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                printf("EMERGENCY STOP\n"); fflush(stdout);
                break;
            }
        }

        /* Final status report after motion completes or STOP */
        printf("STATUS %ld,%ld\n",
            (long)azimuth.position,
            (long)elevation.position);
        fflush(stdout);
        /* Disable motor until next command */
        stepper_close(m);
    }
    return 0;
}

