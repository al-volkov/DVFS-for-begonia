#include "cpufreq_governor.h"

static struct od_ops od_ops;

static unsigned int default_powersave_bias;

struct spsa_core_params {
    unsigned int betta;
    unsigned int alpha;
    unsigned int eta;
    unsigned int gamma;
    unsigned int up;
};

struct spsa_dbs_tuners {
    struct spsa_core_params cluster_0;
    struct spsa_core_params cluster_1;
};

struct spsa_policy_dbs_info
{
    struct policy_dbs_info policy_dbs;

    unsigned int freq_lo;
    unsigned int freq_lo_delay_us;
    unsigned int freq_hi_delay_us;

    signed int delta;
    unsigned int cur_estimation;

    unsigned int sample_type:1;
};

struct od_dbs_tuners {
    unsigned int powersave_bias;
};

static inline struct spsa_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
    return container_of(policy_dbs, struct spsa_policy_dbs_info, policy_dbs);
}

#define gov_show_two(_gov, file_name, num)					\
static ssize_t show_##file_name##_##num					\
(struct gov_attr_set *attr_set, char *buf)				\
{									\
	struct dbs_data *dbs_data = to_dbs_data(attr_set);		\
	struct _gov##_dbs_tuners *tuners = dbs_data->tuners;		\
	return sprintf(buf, "%u\n", tuners->cluster_##num.file_name);			\
}