/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cp_sense, CONFIG_AC_CHARGE_LOG_LEVEL);

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <lib/ac_charge.h>

#define CP_ADC_NODE DT_ALIAS(cp_adc)

#define CP_SAMPLE_RATE_HZ     20000U
#define CP_SAMPLE_COUNT       400U
#define CP_UPDATE_INTERVAL_MS 100U
#define CP_SAMPLE_INTERVAL_US (1000000U / CP_SAMPLE_RATE_HZ)
#define CP_THREAD_STACK_SIZE  3072U
#define CP_THREAD_PRIORITY    8

struct cp_measurement_internal {
    bool valid;
    uint16_t duty_per_mille;
    uint16_t frequency_hz;
    int16_t min_raw;
    int16_t max_raw;
};

/*
 * All mutable module state lives in this context struct. A single static
 * instance is used for the (single) CP channel; passing the context as the
 * first argument to the helpers keeps the implementation testable and avoids
 * scattered file-scope globals.
 */
struct cp_sense_ctx {
    /* Hardware configuration (const after init). */
    const struct adc_dt_spec *adc;

    /* Sampling buffer used by cp_capture_samples / cp_analyze_samples. */
    int16_t *samples;
    size_t sample_count;

    /* Latest decoded measurement, protected by `lock`. */
    struct k_mutex lock;
    struct cp_measurement_internal latest;
    bool has_sample;

    /* Background sampling thread. */
    struct k_thread thread;
    k_thread_stack_t *thread_stack;
    size_t thread_stack_size;
    bool thread_started;
};

#if IS_ENABLED(CONFIG_ADC) && DT_NODE_EXISTS(CP_ADC_NODE)

static const struct adc_dt_spec cp_adc_spec = ADC_DT_SPEC_GET(CP_ADC_NODE);
static int16_t cp_samples[CP_SAMPLE_COUNT];
/*
 * The thread stack must remain at file scope because K_THREAD_STACK_DEFINE
 * places it in a dedicated linker section; only a pointer to it lives in the
 * context struct.
 */
static K_THREAD_STACK_DEFINE(cp_thread_stack, CP_THREAD_STACK_SIZE);

static struct cp_sense_ctx cp_ctx = {
    .adc = &cp_adc_spec,
    .samples = cp_samples,
    .sample_count = CP_SAMPLE_COUNT,
    .thread_stack = cp_thread_stack,
    .thread_stack_size = K_THREAD_STACK_SIZEOF(cp_thread_stack),
};

static int cp_capture_samples(struct cp_sense_ctx *ctx)
{
    struct adc_sequence_options options = {
        .interval_us = CP_SAMPLE_INTERVAL_US,
        .extra_samplings = ctx->sample_count - 1U,
    };

    struct adc_sequence sequence = {
        .buffer = ctx->samples,
        .buffer_size = ctx->sample_count * sizeof(ctx->samples[0]),
        .options = &options,
    };

    int ret = adc_sequence_init_dt(ctx->adc, &sequence);
    if (ret < 0) {
        return ret;
    }

    return adc_read_dt(ctx->adc, &sequence);
}

static void cp_analyze_samples(struct cp_sense_ctx *ctx, struct cp_measurement_internal *out)
{
    const int16_t *samples = ctx->samples;
    const size_t n = ctx->sample_count;

    uint32_t period_sum_us = 0U;
    uint32_t high_sum_us = 0U;
    uint16_t cycles = 0U;

    int16_t min_raw = samples[0];
    int16_t max_raw = samples[0];

    for (size_t i = 1; i < n; i++) {
        if (samples[i] < min_raw) {
            min_raw = samples[i];
        }
        if (samples[i] > max_raw) {
            max_raw = samples[i];
        }
    }

    int32_t span = (int32_t)max_raw - (int32_t)min_raw;
    if (span < 64) {
        out->valid = false;
        out->min_raw = min_raw;
        out->max_raw = max_raw;
        return;
    }

    int16_t low_thr = (int16_t)(min_raw + (span * 40) / 100);
    int16_t high_thr = (int16_t)(min_raw + (span * 60) / 100);

    bool is_high = samples[0] >= high_thr;
    uint32_t last_rise_us = 0U;
    uint32_t last_fall_us = 0U;
    bool have_rise = false;
    bool have_fall = false;

    const uint32_t dt_us = 1000000U / CP_SAMPLE_RATE_HZ;

    for (size_t i = 1; i < n; i++) {
        const int16_t s = samples[i];
        const uint32_t t_us = (uint32_t)i * dt_us;

        if (!is_high && (s >= high_thr)) {
            if (have_rise && have_fall && (t_us > last_rise_us)) {
                uint32_t period_us = t_us - last_rise_us;
                uint32_t high_us = last_fall_us - last_rise_us;

                if ((high_us > 0U) && (period_us > high_us)) {
                    period_sum_us += period_us;
                    high_sum_us += high_us;
                    cycles++;
                }
            }

            last_rise_us = t_us;
            have_rise = true;
            have_fall = false;
            is_high = true;
        } else if (is_high && (s <= low_thr)) {
            last_fall_us = t_us;
            have_fall = true;
            is_high = false;
        }
    }

    out->min_raw = min_raw;
    out->max_raw = max_raw;

    if (cycles == 0U) {
        out->valid = false;
        return;
    }

    uint32_t avg_period_us = period_sum_us / cycles;
    uint32_t avg_high_us = high_sum_us / cycles;
    uint32_t freq_hz = 1000000U / MAX(avg_period_us, 1U);
    uint32_t duty_per_mille = (avg_high_us * 1000U) / MAX(avg_period_us, 1U);

    out->frequency_hz = (uint16_t)MIN(freq_hz, UINT16_MAX);
    out->duty_per_mille = (uint16_t)MIN(duty_per_mille, 1000U);
    out->valid = true;
}

static void cp_measure_once(struct cp_sense_ctx *ctx)
{
    struct cp_measurement_internal next = {0};

    int ret = cp_capture_samples(ctx);
    if (ret < 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return;
    }

    cp_analyze_samples(ctx, &next);
    int32_t min_mv = next.min_raw;
    int32_t max_mv = next.max_raw;
    (void)adc_raw_to_millivolts_dt(ctx->adc, &min_mv);
    (void)adc_raw_to_millivolts_dt(ctx->adc, &max_mv);

    k_mutex_lock(&ctx->lock, K_FOREVER);
    ctx->latest = next;
    ctx->has_sample = true;
    k_mutex_unlock(&ctx->lock);

    LOG_DBG("CP: %u Hz, %u.%u%%, raw[%d..%d], mv[%d..%d]", next.frequency_hz,
            next.duty_per_mille / 10U, next.duty_per_mille % 10U, next.min_raw, next.max_raw,
            (int)min_mv, (int)max_mv);
}

static void cp_measure_thread(void *arg1, void *arg2, void *arg3)
{
    struct cp_sense_ctx *ctx = arg1;
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true) {
        cp_measure_once(ctx);
        k_sleep(K_MSEC(CP_UPDATE_INTERVAL_MS));
    }
}

int cp_sense_init(void)
{
    struct cp_sense_ctx *ctx = &cp_ctx;

    k_mutex_init(&ctx->lock);

    if (!adc_is_ready_dt(ctx->adc)) {
        LOG_ERR("CP ADC device not ready (alias: cp-adc)");
        return -ENODEV;
    }

    int ret = adc_channel_setup_dt(ctx->adc);
    if (ret < 0) {
        LOG_ERR("CP ADC channel setup failed: %d", ret);
        return ret;
    }

    if (!ctx->thread_started) {
        k_tid_t tid = k_thread_create(&ctx->thread, ctx->thread_stack, ctx->thread_stack_size,
                                      cp_measure_thread, ctx, NULL, NULL,
                                      K_PRIO_PREEMPT(CP_THREAD_PRIORITY), 0, K_NO_WAIT);
        k_thread_name_set(tid, "cp_sense");
        ctx->thread_started = true;
    }

    LOG_INF("CP sensing started (%u ksps, %u ms window)", CP_SAMPLE_RATE_HZ / 1000U,
            (1000U * CP_SAMPLE_COUNT) / CP_SAMPLE_RATE_HZ);

    return 0;
}

int cp_sense_get_latest(struct cp_measurement *out)
{
    if (out == NULL) {
        return -EINVAL;
    }

    struct cp_sense_ctx *ctx = &cp_ctx;

    k_mutex_lock(&ctx->lock, K_FOREVER);
    bool has_sample = ctx->has_sample;
    out->valid = ctx->latest.valid;
    out->duty_per_mille = ctx->latest.duty_per_mille;
    out->frequency_hz = ctx->latest.frequency_hz;
    out->min_raw = ctx->latest.min_raw;
    out->max_raw = ctx->latest.max_raw;
    k_mutex_unlock(&ctx->lock);

    return has_sample ? 0 : -EAGAIN;
}

#else

int cp_sense_init(void)
{
    LOG_WRN("CP sensing disabled: enable CONFIG_ADC and DT alias 'cp-adc'");
    return -ENOTSUP;
}

int cp_sense_get_latest(struct cp_measurement *out)
{
    ARG_UNUSED(out);
    return -ENOTSUP;
}

#endif
