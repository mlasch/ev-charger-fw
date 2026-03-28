/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

struct cp_measurement {
    bool valid;
    uint16_t duty_per_mille;
    uint16_t frequency_hz;
    int16_t min_raw;
    int16_t max_raw;
};

/**
 * Set PWM duty cycle
 * @param percent
 * @return
 */
int set_duty_cycle_percent(uint8_t percent);

/**
 * Init AC charge
 * @return
 */
int init_ac_charge(void);

/**
 * Init Control Pilot ADC sensing.
 * @return 0 on success, negative errno on error.
 */
int cp_sense_init(void);

/**
 * Get latest CP measurement.
 * @param out destination structure.
 * @return 0 when a valid sample is available, -EAGAIN if not yet available.
 */
int cp_sense_get_latest(struct cp_measurement *out);
