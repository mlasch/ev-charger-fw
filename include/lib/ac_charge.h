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

enum ac_charge_state {
    AC_CHARGE_STATE_IDLE = 0,
    AC_CHARGE_STATE_CONNECTED,
    AC_CHARGE_STATE_READY,
    AC_CHARGE_STATE_VENTILATION,
    AC_CHARGE_STATE_FAULT,
};

enum ac_charge_event {
    AC_CHARGE_EVENT_NONE = 0,
    AC_CHARGE_EVENT_EV_CONNECTED,
    AC_CHARGE_EVENT_EV_DISCONNECTED,
    AC_CHARGE_EVENT_EV_READY,
    AC_CHARGE_EVENT_EV_NOT_READY,
    AC_CHARGE_EVENT_VENTILATION_REQUIRED,
    AC_CHARGE_EVENT_VENTILATION_CLEARED,
    AC_CHARGE_EVENT_FAULT_DETECTED,
    AC_CHARGE_EVENT_FAULT_CLEARED,
    AC_CHARGE_EVENT_COUNT,
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

/**
 * Initialize AC charge state machine.
 */
int ac_charge_sm_init(void);

/**
 * Post an external event to the state machine.
 */
int ac_charge_sm_post_event(enum ac_charge_event event);

/**
 * Execute one state machine step.
 */
int ac_charge_sm_step(void);

/**
 * Read current state.
 */
enum ac_charge_state ac_charge_sm_get_state(void);

/**
 * Optional helper: post transitions derived from CP measurement.
 */
void ac_charge_sm_update_from_cp(const struct cp_measurement *cp);
