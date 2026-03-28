/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm, CONFIG_AC_CHARGE_LOG_LEVEL);

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <lib/ac_charge.h>

static const struct pwm_dt_spec cp_pwm = PWM_DT_SPEC_GET(DT_ALIAS(cp_pwm));

#define PWM_PERIOD_NS 1000000U /* 1 ms = 1 kHz */

int set_duty_cycle_percent(uint8_t percent)
{
    if (percent > 100) {
        return -EINVAL;
    }

    uint32_t pulse_ns = (PWM_PERIOD_NS * percent) / 100U;

    int ret = pwm_set_dt(&cp_pwm, PWM_PERIOD_NS, pulse_ns);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM: %d", ret);
    }
    return ret;
}

int init_ac_charge(void)
{
    int ret;
    if (!pwm_is_ready_dt(&cp_pwm)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    ret = cp_sense_init();
    if ((ret < 0) && (ret != -ENOTSUP)) {
        return ret;
    }

    ret = ac_charge_sm_init();
    if (ret < 0) {
        return ret;
    }

    return 0;
}
