/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm, CONFIG_AC_CHARGE_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_out = PWM_DT_SPEC_GET(DT_ALIAS(pwm_out));

#define PWM_PERIOD_NS 1000000U /* 1 ms = 1 kHz */

int set_duty_cycle_percent(uint8_t percent)
{
    if (percent > 100) {
        return -EINVAL;
    }

    uint32_t pulse_ns = (PWM_PERIOD_NS * percent) / 100U;

    int ret = pwm_set_dt(&pwm_out, PWM_PERIOD_NS, pulse_ns);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM: %d", ret);
    }
    return ret;
}

int init_ac_charge(void)
{
    if (!pwm_is_ready_dt(&pwm_out)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    LOG_INF("Starting 1 kHz PWM at 50%% duty cycle");
    int ret = set_duty_cycle_percent(50);
    if (ret < 0) {
        return ret;
    }

    return 0;
}
