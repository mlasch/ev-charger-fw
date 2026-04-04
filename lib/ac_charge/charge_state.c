/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(charge_state, CONFIG_AC_CHARGE_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/smf.h>

#include <lib/ac_charge.h>

struct ac_sm_ctx {
    struct smf_ctx smf;
    enum ac_charge_state state;
    enum ac_charge_event pending_event;
    bool pending_event_set;
};

enum ac_sm_idx {
    AC_SM_IDLE,
    AC_SM_CONNECTED,
    AC_SM_READY,
    AC_SM_VENTILATION,
    AC_SM_FAULT,
    AC_SM_COUNT,
};

static struct ac_sm_ctx g_sm;
static K_MUTEX_DEFINE(g_sm_lock);

static const struct smf_state ac_states[AC_SM_COUNT];

static enum ac_charge_event ac_take_event_locked(void)
{
    if (!g_sm.pending_event_set) {
        return AC_CHARGE_EVENT_NONE;
    }

    enum ac_charge_event ev = g_sm.pending_event;
    g_sm.pending_event_set = false;
    g_sm.pending_event = AC_CHARGE_EVENT_NONE;
    return ev;
}

static void ac_set_state(struct ac_sm_ctx *ctx, enum ac_sm_idx idx)
{
    smf_set_state(SMF_CTX(ctx), &ac_states[idx]);
}

static void ac_idle_entry(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    ctx->state = AC_CHARGE_STATE_IDLE;
    int ret = set_duty_cycle_percent(100);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM: %d", ret);
    }
    LOG_INF("State -> IDLE");
}

static enum smf_state_result ac_idle_run(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    enum ac_charge_event ev = ac_take_event_locked();

    switch (ev) {
    case AC_CHARGE_EVENT_EV_CONNECTED:
        ac_set_state(ctx, AC_SM_CONNECTED);
        break;
    case AC_CHARGE_EVENT_FAULT_DETECTED:
        ac_set_state(ctx, AC_SM_FAULT);
        break;
    default:
        break;
    }

    return SMF_EVENT_HANDLED;
}

static void ac_connected_entry(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    ctx->state = AC_CHARGE_STATE_CONNECTED;
    int ret = set_duty_cycle_percent(50);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM: %d", ret);
    }
    LOG_INF("State -> CONNECTED");
}

static enum smf_state_result ac_connected_run(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    enum ac_charge_event ev = ac_take_event_locked();

    switch (ev) {
    case AC_CHARGE_EVENT_EV_DISCONNECTED:
        ac_set_state(ctx, AC_SM_IDLE);
        break;
    case AC_CHARGE_EVENT_EV_READY:
        ac_set_state(ctx, AC_SM_READY);
        break;
    case AC_CHARGE_EVENT_VENTILATION_REQUIRED:
        ac_set_state(ctx, AC_SM_VENTILATION);
        break;
    case AC_CHARGE_EVENT_FAULT_DETECTED:
        ac_set_state(ctx, AC_SM_FAULT);
        break;
    default:
        break;
    }

    return SMF_EVENT_HANDLED;
}

static void ac_ready_entry(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    ctx->state = AC_CHARGE_STATE_READY;
    LOG_INF("State -> READY");
}

static enum smf_state_result ac_ready_run(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    enum ac_charge_event ev = ac_take_event_locked();

    switch (ev) {
    case AC_CHARGE_EVENT_EV_DISCONNECTED:
        ac_set_state(ctx, AC_SM_IDLE);
        break;
    case AC_CHARGE_EVENT_VENTILATION_REQUIRED:
        ac_set_state(ctx, AC_SM_VENTILATION);
        break;
    case AC_CHARGE_EVENT_EV_NOT_READY:
        ac_set_state(ctx, AC_SM_CONNECTED);
        break;
    case AC_CHARGE_EVENT_FAULT_DETECTED:
        ac_set_state(ctx, AC_SM_FAULT);
        break;
    default:
        break;
    }

    return SMF_EVENT_HANDLED;
}

static void ac_vent_entry(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    ctx->state = AC_CHARGE_STATE_VENTILATION;
    LOG_INF("State -> VENTILATION");
}

static enum smf_state_result ac_vent_run(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    enum ac_charge_event ev = ac_take_event_locked();

    switch (ev) {
    case AC_CHARGE_EVENT_EV_DISCONNECTED:
        ac_set_state(ctx, AC_SM_IDLE);
        break;
    case AC_CHARGE_EVENT_VENTILATION_CLEARED:
        ac_set_state(ctx, AC_SM_CONNECTED);
        break;
    case AC_CHARGE_EVENT_FAULT_DETECTED:
        ac_set_state(ctx, AC_SM_FAULT);
        break;
    default:
        break;
    }

    return SMF_EVENT_HANDLED;
}

static void ac_fault_entry(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    ctx->state = AC_CHARGE_STATE_FAULT;
    LOG_INF("State -> FAULT");
}

static enum smf_state_result ac_fault_run(void *obj)
{
    struct ac_sm_ctx *ctx = obj;
    enum ac_charge_event ev = ac_take_event_locked();

    if (ev == AC_CHARGE_EVENT_FAULT_CLEARED) {
        ac_set_state(ctx, AC_SM_IDLE);
    }

    return SMF_EVENT_HANDLED;
}

static const struct smf_state ac_states[AC_SM_COUNT] = {
    [AC_SM_IDLE] = SMF_CREATE_STATE(ac_idle_entry, ac_idle_run, NULL, NULL, NULL),
    [AC_SM_CONNECTED] = SMF_CREATE_STATE(ac_connected_entry, ac_connected_run, NULL, NULL, NULL),
    [AC_SM_READY] = SMF_CREATE_STATE(ac_ready_entry, ac_ready_run, NULL, NULL, NULL),
    [AC_SM_VENTILATION] = SMF_CREATE_STATE(ac_vent_entry, ac_vent_run, NULL, NULL, NULL),
    [AC_SM_FAULT] = SMF_CREATE_STATE(ac_fault_entry, ac_fault_run, NULL, NULL, NULL),
};

int ac_charge_sm_init(void)
{
    k_mutex_lock(&g_sm_lock, K_FOREVER);
    g_sm.pending_event = AC_CHARGE_EVENT_NONE;
    g_sm.pending_event_set = false;
    smf_set_initial(SMF_CTX(&g_sm), &ac_states[AC_SM_IDLE]);
    k_mutex_unlock(&g_sm_lock);

    return 0;
}

int ac_charge_sm_post_event(enum ac_charge_event event)
{
    if ((event <= AC_CHARGE_EVENT_NONE) || (event >= AC_CHARGE_EVENT_COUNT)) {
        return -EINVAL;
    }

    k_mutex_lock(&g_sm_lock, K_FOREVER);
    g_sm.pending_event = event;
    g_sm.pending_event_set = true;
    k_mutex_unlock(&g_sm_lock);

    return 0;
}

int ac_charge_sm_step(void)
{
    k_mutex_lock(&g_sm_lock, K_FOREVER);
    int ret = smf_run_state(SMF_CTX(&g_sm));
    k_mutex_unlock(&g_sm_lock);

    return ret;
}

enum ac_charge_state ac_charge_sm_get_state(void)
{
    k_mutex_lock(&g_sm_lock, K_FOREVER);
    enum ac_charge_state state = g_sm.state;
    k_mutex_unlock(&g_sm_lock);

    return state;
}

void ac_charge_sm_update_from_cp(const struct cp_measurement *cp)
{
    if (cp == NULL) {
        return;
    }

    /*
     * Pick a representative CP voltage sample.
     *
     * IEC 61851 state A (no vehicle) and state B (vehicle connected) are both
     * static DC levels (+12 V and +9 V respectively); the charger only starts
     * the 1 kHz PWM after detecting the vehicle. As a result, on the very
     * first connect the CP signal has no PWM and cp->valid is false, but the
     * captured min/max raw samples still reflect the DC level.
     *
     * - When PWM is present, the positive plateau (max_raw) encodes the state.
     * - When PWM is absent (DC), min_raw ~= max_raw, so the average works for
     *   both cases and lets us transition out of IDLE on first connect.
     */
    int32_t level_raw;
    if (cp->valid) {
        level_raw = cp->max_raw;
    } else {
        level_raw = ((int32_t)cp->min_raw + (int32_t)cp->max_raw) / 2;
    }

    /*
     * Best-effort CP interpretation using the conditioned 0..3V signal.
     * Approximate mapping used for transitions (raw ADC counts):
     *  - >3700  -> +12V  : disconnected (state A)
     *  - >3400  -> +9V   : connected    (state B)
     *  - >2800  -> +6V   : ready        (state C)
     *  - >2100  -> +3V   : ventilation  (state D)
     *  - else           : fault        (state E/F)
     */
    if (level_raw > 3700) {
        (void)ac_charge_sm_post_event(AC_CHARGE_EVENT_EV_DISCONNECTED);
    } else if (level_raw > 3400) {
        (void)ac_charge_sm_post_event(AC_CHARGE_EVENT_EV_CONNECTED);
    } else if (level_raw > 2800) {
        (void)ac_charge_sm_post_event(AC_CHARGE_EVENT_EV_READY);
    } else if (level_raw > 2100) {
        (void)ac_charge_sm_post_event(AC_CHARGE_EVENT_VENTILATION_REQUIRED);
    } else {
        (void)ac_charge_sm_post_event(AC_CHARGE_EVENT_FAULT_DETECTED);
    }
}
