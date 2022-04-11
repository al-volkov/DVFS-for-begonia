/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/sched/cpufreq.h>
#include <linux/prandom.h>
#include <linux/gcd.h>

#include <linux/kernel.h>

#include "cpufreq_spsa2.h"

#include <linux/ktime.h>
#include <linux/timekeeping.h>

#define DEF_UP_THRESHOLD            (64)
#define DEF_ALPHA_VALUE                (26000)
#define DEF_BETTA_VALUE                (26000)
#define DEF_ALPHA_VALUE_BIG                (31000)
#define DEF_BETTA_VALUE_BIG                (31000)
#define START_FREQUENCY_ESTIMATION        (500000)
#define MIN_FREQUENCY_UP_THRESHOLD        (70)
#define MAX_FREQUENCY_UP_THRESHOLD        (100)
//------------------------
#define DEF_SAMPLING_DOWN_FACTOR        (1)
#define MAX_SAMPLING_DOWN_FACTOR        (100000)


/*
 * Not all CPUs want IO time to be accounted as busy; this depends on how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (android.com) claims this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
    /*
     * For Intel, Core 2 (model 15) and later have an efficient idle.
     */
    if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
            boot_cpu_data.x86 == 6 &&
            boot_cpu_data.x86_model >= 15)
        return 1;
#endif
    return 0;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_delay_us,
 * freq_lo, and freq_lo_delay_us in percpu area for averaging freqs.
 */
static unsigned int generic_powersave_bias_target(struct cpufreq_policy* policy,
                                                  unsigned int freq_next, unsigned int relation)
{
    unsigned int freq_req, freq_reduc, freq_avg;
    unsigned int freq_hi, freq_lo;
    unsigned int index;
    unsigned int delay_hi_us;
    struct policy_dbs_info* policy_dbs = policy->governor_data;
    struct spsa_policy_dbs_info* dbs_info = to_dbs_info(policy_dbs);
    struct dbs_data* dbs_data = policy_dbs->dbs_data;
    struct od_dbs_tuners* od_tuners = dbs_data->tuners;
    struct cpufreq_frequency_table* freq_table = policy->freq_table;

    if (!freq_table) {
        dbs_info->freq_lo = 0;
        dbs_info->freq_lo_delay_us = 0;
        return freq_next;
    }

    index = cpufreq_frequency_table_target(policy, freq_next, relation);
    freq_req = freq_table[index].frequency;
    freq_reduc = freq_req * od_tuners->powersave_bias / 1000;
    freq_avg = freq_req - freq_reduc;

    /* Find freq bounds for freq_avg in freq_table */
    index = cpufreq_table_find_index_h(policy, freq_avg);
    freq_lo = freq_table[index].frequency;
    index = cpufreq_table_find_index_l(policy, freq_avg);
    freq_hi = freq_table[index].frequency;

    /* Find out how long we have to be in hi and lo freqs */
    if (freq_hi == freq_lo) {
        dbs_info->freq_lo = 0;
        dbs_info->freq_lo_delay_us = 0;
        return freq_lo;
    }
    delay_hi_us = (freq_avg - freq_lo) * dbs_data->sampling_rate;
    delay_hi_us += (freq_hi - freq_lo) / 2;
    delay_hi_us /= freq_hi - freq_lo;
    dbs_info->freq_hi_delay_us = delay_hi_us;
    dbs_info->freq_lo = freq_lo;
    dbs_info->freq_lo_delay_us = dbs_data->sampling_rate - delay_hi_us;
    return freq_hi;
}

static void ondemand_powersave_bias_init(struct cpufreq_policy* policy)
{
    struct spsa_policy_dbs_info* dbs_info = to_dbs_info(policy->governor_data);

    dbs_info->freq_lo = 0;
}

static signed int generate_delta()
{

    if (prandom_u32() & 0x1) {
        return 1;
    }

    return -1;
}

static unsigned int freq_1[16] = {
        2000000,
        1933000,
        1866000,
        1800000,
        1733000,
        1666000,
        1618000,
        1500000,
        1375000,
        1275000,
        1175000,
        1075000,
        975000,
        875000,
        774000,
        500000
};

static unsigned int freq_2[16] = {
        2050000,
        1986000,
        1923000,
        1860000,
        1796000,
        1733000,
        1670000,
        1530000,
        1419000,
        1308000,
        1169000,
        1085000,
        1002000,
        919000,
        835000,
        774000
};

static unsigned int weights[16] = {
        437893,
        291929,
        194619,
        129746,
        86497,
        57665,
        38443,
        25628,
        17085,
        11390,
        7593,
        5062,
        3375,
        2250,
        1500,
        1000
};


static unsigned int find(unsigned int* freq, unsigned int val)
{
    for (int i = 0; i < 16; i++) {
        if (val == freq[i]) {
            return i;
        }
    }

    return -1;
}

static unsigned int get_val(unsigned int freq, int cpu, unsigned int load,
                            struct spsa_core_params* spsa_tuners)
{

    unsigned int index = 0;
    unsigned int weight = 0;

    if (cpu < 6) {
        index = find(freq_1, freq);
    } else {
        index = find(freq_2, freq);
    }

    weight = weights[index];

    return weight;
}

static u64 model_count(unsigned int freq, unsigned int load,
                       struct spsa_core_params* spsa_tuners, int cpu)
{
    u64 model_val = 0;

    if (load > spsa_tuners->up) {
        model_val = 2 << ((load - spsa_tuners->up) / 2);
    }

    model_val += get_val(freq, cpu, load, spsa_tuners);

    return model_val;
}

static struct spsa_core_params* get_spsa_core_params(unsigned int cpu, struct spsa_dbs_tuners* spsa_tuners)
{

    if (cpu < 6) {
        return &spsa_tuners->cluster_0;
    }

    return &spsa_tuners->cluster_1;
}

// 0 - without addition, 1 - with betta * delta
static unsigned int spsa_phase = 0;

static u64 old_model = 0;

static void log_spsa(int cpu, unsigned int load, unsigned int freq)
{

    pr_alert("gov spsa2, cpu %d, freq %u, load is %u", cpu, freq, load);
}

static unsigned int find_closest(unsigned int frequency, unsigned int cpu)
{
    unsigned int* freq;
    unsigned int closest;
    int i;

    if (cpu < 6) {
        freq = freq_1;
    } else {
        freq = freq_2;
    }

    closest = freq[0];

    for (i = 0; i < 16; i++) {
        if (abs(freq[i] - frequency) < abs(freq[i] - closest)) {
            closest = freq[i];
        }
    }
    pr_alert("freq - %u closeset - %u, cpu - %u", frequency, closest, cpu);

    return closest;


}

static void od_update(struct cpufreq_policy* policy)
{
    struct policy_dbs_info* policy_dbs = policy->governor_data;
    struct spsa_policy_dbs_info* dbs_info = to_dbs_info(policy_dbs);
    struct dbs_data* dbs_data = policy_dbs->dbs_data;
    struct spsa_core_params* spsa_tuners = get_spsa_core_params(policy->cpu, dbs_data->tuners);

    unsigned int freq_next;
    unsigned int index;
    unsigned int load = dbs_update(policy);

    log_spsa(policy->cpu, load, policy->cur);

    u64 model = model_count(policy->cur, load, spsa_tuners, policy->cpu);
    unsigned int gcd_val = gcd(spsa_tuners->alpha, spsa_tuners->betta);

    unsigned int norm_alpha = spsa_tuners->alpha / gcd_val;
    unsigned int norm_betta = spsa_tuners->betta / gcd_val;

    if (spsa_phase) {
        dbs_info->cur_estimation = dbs_info->cur_estimation - ((model - old_model) / norm_betta) * norm_alpha * dbs_info->delta;
        dbs_info->cur_estimation = find_closest(dbs_info->cur_estimation, policy->cpu);
    } else {
        old_model = model;
    }
    spsa_phase = !spsa_phase;

    if (dbs_info->cur_estimation > policy->max) {
        dbs_info->cur_estimation = policy->max;
    }

    if (dbs_info->cur_estimation < policy->min) {
        dbs_info->cur_estimation = policy->min;
    }

    freq_next = dbs_info->cur_estimation;
    if (spsa_phase) {
        dbs_info->delta = generate_delta();
        freq_next += spsa_tuners->betta * dbs_info->delta;
        freq_next = find_closest(freq_next, policy->cpu);
    }

    dbs_info->freq_lo = 0;
    policy_dbs->rate_mult = 1;

    __cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
}

static unsigned int od_dbs_update(struct cpufreq_policy* policy)
{
    struct policy_dbs_info* policy_dbs = policy->governor_data;
    struct dbs_data* dbs_data = policy_dbs->dbs_data;
    struct spsa_policy_dbs_info* dbs_info = to_dbs_info(policy_dbs);
    int sample_type = dbs_info->sample_type;

    /* Common NORMAL_SAMPLE setup */
    dbs_info->sample_type = OD_NORMAL_SAMPLE;
    /*
     * OD_SUB_SAMPLE doesn't make sense if sample_delay_ns is 0, so ignore
     * it then.
     */
    if (sample_type == OD_SUB_SAMPLE && policy_dbs->sample_delay_ns > 0) {
        __cpufreq_driver_target(policy, dbs_info->freq_lo,
                                CPUFREQ_RELATION_H);
        return dbs_info->freq_lo_delay_us;
    }

    od_update(policy);

    if (dbs_info->freq_lo) {
        /* Setup SUB_SAMPLE */
        dbs_info->sample_type = OD_SUB_SAMPLE;
        return dbs_info->freq_hi_delay_us;
    }

    return dbs_data->sampling_rate * policy_dbs->rate_mult;
}

/************************** sysfs interface ************************/
static struct dbs_governor od_dbs_gov;

static ssize_t store_io_is_busy(struct gov_attr_set* attr_set, const char* buf,
                                size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    unsigned int input;
    int ret;

    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;
    dbs_data->io_is_busy = !!input;

    /* we need to re-evaluate prev_cpu_idle */
    gov_update_cpu_data(dbs_data);

    return count;
}

static ssize_t store_up_threshold(struct gov_attr_set* attr_set,
                                  const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
        input < MIN_FREQUENCY_UP_THRESHOLD) {
        return -EINVAL;
    }

    dbs_data->up_threshold = input;
    return count;
}

static ssize_t store_betta_0(struct gov_attr_set* attr_set,
                             const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_0.betta = input;
    return count;
}

static ssize_t store_betta_1(struct gov_attr_set* attr_set,
                             const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_1.betta = input;
    return count;
}

static ssize_t store_alpha_0(struct gov_attr_set* attr_set,
                             const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;

    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_0.alpha = input;

    return count;
}

static ssize_t store_alpha_1(struct gov_attr_set* attr_set,
                             const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;

    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_1.alpha = input;

    return count;
}

static ssize_t store_up_0(struct gov_attr_set* attr_set,
                          const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_0.up = input;
    return count;
}

static ssize_t store_up_1(struct gov_attr_set* attr_set,
                          const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct spsa_dbs_tuners* spsa_tuners = dbs_data->tuners;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1) {
        return -EINVAL;
    }

    spsa_tuners->cluster_1.up = input;
    return count;
}

static ssize_t store_sampling_down_factor(struct gov_attr_set* attr_set,
                                          const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct policy_dbs_info* policy_dbs;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
        return -EINVAL;

    dbs_data->sampling_down_factor = input;

    /* Reset down sampling multiplier in case it was active */
    list_for_each_entry(policy_dbs, &attr_set->policy_list, list)
    {
        /*
         * Doing this without locking might lead to using different
         * rate_mult values in od_update() and od_dbs_update().
         */
        mutex_lock(&policy_dbs->update_mutex);
        policy_dbs->rate_mult = 1;
        mutex_unlock(&policy_dbs->update_mutex);
    }

    return count;
}

static ssize_t store_ignore_nice_load(struct gov_attr_set* attr_set,
                                      const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    unsigned int input;
    int ret;

    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    if (input > 1)
        input = 1;

    if (input == dbs_data->ignore_nice_load) { /* nothing to do */
        return count;
    }
    dbs_data->ignore_nice_load = input;

    /* we need to re-evaluate prev_cpu_idle */
    gov_update_cpu_data(dbs_data);

    return count;
}

static ssize_t store_powersave_bias(struct gov_attr_set* attr_set,
                                    const char* buf, size_t count)
{
    struct dbs_data* dbs_data = to_dbs_data(attr_set);
    struct od_dbs_tuners* od_tuners = dbs_data->tuners;
    struct policy_dbs_info* policy_dbs;
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1)
        return -EINVAL;

    if (input > 1000)
        input = 1000;

    od_tuners->powersave_bias = input;

    list_for_each_entry(policy_dbs, &attr_set->policy_list, list)
    ondemand_powersave_bias_init(policy_dbs->policy);

    return count;
}

gov_show_one_common(sampling_rate);

gov_show_one_common(up_threshold);

gov_show_one_common(sampling_down_factor);

gov_show_one_common(ignore_nice_load);

gov_show_one_common(io_is_busy);

gov_show_one(od, powersave_bias);

gov_show_two(spsa, betta, 0);

gov_show_two(spsa, betta, 1);

gov_show_two(spsa, alpha, 0);

gov_show_two(spsa, alpha, 1);

gov_show_two(spsa, up, 0);

gov_show_two(spsa, up, 1);

gov_attr_rw(sampling_rate);
gov_attr_rw(io_is_busy);
gov_attr_rw(up_threshold);
gov_attr_rw(betta_0);
gov_attr_rw(betta_1);
gov_attr_rw(up_0);
gov_attr_rw(up_1);
gov_attr_rw(alpha_0);
gov_attr_rw(alpha_1);
gov_attr_rw(sampling_down_factor);
gov_attr_rw(ignore_nice_load);
gov_attr_rw(powersave_bias);

static struct attribute* od_attributes[] = {
        &sampling_rate.attr,
        &up_threshold.attr,
        &betta_0.attr,
        &betta_1.attr,
        &alpha_0.attr,
        &alpha_1.attr,
        &up_0.attr,
        &up_1.attr,
        &sampling_down_factor.attr,
        &ignore_nice_load.attr,
        &powersave_bias.attr,
        &io_is_busy.attr,
        NULL
};

/************************** sysfs end ************************/

static struct policy_dbs_info* od_alloc(void)
{
    struct spsa_policy_dbs_info* dbs_info;

    dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
    return dbs_info ? &dbs_info->policy_dbs : NULL;
}

static void od_free(struct policy_dbs_info* policy_dbs)
{
    pr_alert("End\n");
    kfree(to_dbs_info(policy_dbs));
}

static int od_init(struct dbs_data* dbs_data)
{
    struct spsa_dbs_tuners* tuners;
    u64 idle_time;
    int cpu;

    tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
    if (!tuners)
        return -ENOMEM;

    cpu = get_cpu();

    idle_time = get_cpu_idle_time_us(cpu, NULL);
    put_cpu();
    dbs_data->up_threshold = DEF_UP_THRESHOLD;

    dbs_data->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
    dbs_data->ignore_nice_load = 0;
    dbs_data->io_is_busy = should_io_be_busy();

    tuners->cluster_0.alpha = DEF_ALPHA_VALUE;
    tuners->cluster_1.alpha = DEF_ALPHA_VALUE_BIG;
    tuners->cluster_0.betta = DEF_BETTA_VALUE;
    tuners->cluster_1.betta = DEF_BETTA_VALUE_BIG;
    tuners->cluster_0.up = DEF_UP_THRESHOLD;
    tuners->cluster_1.up = DEF_UP_THRESHOLD;

    dbs_data->tuners = tuners;
    return 0;
}

static void od_exit(struct dbs_data* dbs_data)
{
    kfree(dbs_data->tuners);
}

static void od_start(struct cpufreq_policy* policy)
{
    struct spsa_policy_dbs_info* dbs_info = to_dbs_info(policy->governor_data);

    dbs_info->sample_type = OD_NORMAL_SAMPLE;

    dbs_info->delta = 1;
    dbs_info->cur_estimation = START_FREQUENCY_ESTIMATION;

    ondemand_powersave_bias_init(policy);
}

static struct od_ops od_ops = {
        .powersave_bias_target = generic_powersave_bias_target,
};

static struct dbs_governor od_dbs_gov = {
        .gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("spsa2"),
        .kobj_type = {.default_attrs = od_attributes},
        .gov_dbs_update = od_dbs_update,
        .alloc = od_alloc,
        .free = od_free,
        .init = od_init,
        .exit = od_exit,
        .start = od_start,
};

#define CPU_FREQ_GOV_SPSA2    (&od_dbs_gov.gov)

static void od_set_powersave_bias(unsigned int powersave_bias)
{
    unsigned int cpu;
    cpumask_t done;

    default_powersave_bias = powersave_bias;
    cpumask_clear(&done);

    get_online_cpus();
    for_each_online_cpu(cpu)
    {
        struct cpufreq_policy* policy;
        struct policy_dbs_info* policy_dbs;
        struct dbs_data* dbs_data;
        struct od_dbs_tuners* od_tuners;

        if (cpumask_test_cpu(cpu, &done))
            continue;

        policy = cpufreq_cpu_get_raw(cpu);
        if (!policy || policy->governor != CPU_FREQ_GOV_SPSA2)
            continue;

        policy_dbs = policy->governor_data;
        if (!policy_dbs)
            continue;

        cpumask_or(&done, &done, policy->cpus);

        dbs_data = policy_dbs->dbs_data;
        od_tuners = dbs_data->tuners;
        od_tuners->powersave_bias = default_powersave_bias;
    }
    put_online_cpus();
}

void od_register_powersave_bias_handler_copy2(unsigned int (* f)
        (struct cpufreq_policy*, unsigned int, unsigned int),
                                              unsigned int powersave_bias)
{
    od_ops.powersave_bias_target = f;
    od_set_powersave_bias(powersave_bias);
}

EXPORT_SYMBOL_GPL(od_register_powersave_bias_handler_copy2);

void od_unregister_powersave_bias_handler_copy2(void)
{
    od_ops.powersave_bias_target = generic_powersave_bias_target;
    od_set_powersave_bias(0);
}

EXPORT_SYMBOL_GPL(od_unregister_powersave_bias_handler_copy2);

static int __init

cpufreq_gov_dbs_init(void)
{
    return cpufreq_register_governor(CPU_FREQ_GOV_SPSA2);
}

static void __exit

cpufreq_gov_dbs_exit(void)
{
    cpufreq_unregister_governor(CPU_FREQ_GOV_SPSA2);
}

MODULE_AUTHOR("Bogdanov Evgenii <gekabog@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_spsa' - A dynamic cpufreq governor for "
"Low Latency Frequency Transition capable processors based on SPSA algorithm");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SPSA2
struct cpufreq_governor *cpufreq_default_governor(void)
{
    return CPU_FREQ_GOV_SPSA2;
}

fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
