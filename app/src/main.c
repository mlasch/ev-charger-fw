/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include <lib/ac_charge.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
    int ret;
    struct cp_measurement cp;
    LOG_INF("Start ac_charge, version %s", APP_VERSION_STRING);

    ret = init_ac_charge();
    if (ret < 0) {
        LOG_ERR("init ac_charge failed, ret: %d", ret);
    }

    while (1) {
        if (cp_sense_get_latest(&cp) == 0) {
            ac_charge_sm_update_from_cp(&cp);
        }

        ret = ac_charge_sm_step();
        if (ret < 0) {
            LOG_ERR("ac_charge_sm_step failed: %d", ret);
        }

        k_sleep(K_SECONDS(1));
    }
}
