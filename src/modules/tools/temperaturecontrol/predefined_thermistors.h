
typedef struct {
        const char *name;
        float beta;
        float r0;
        float t0;
        int r1;
        int r2;
} const thermistor_table_t;

static const thermistor_table_t predefined_thermistors[] {
    // name,      beta,    r0,        t0,   r1, r2
    {"EPCOS100K", 4066.0F, 100000.0F, 25.0F, 0, 4700},
    {"RRRF100K", 3960.0F, 100000.0F, 25.0F, 0, 4700},
    {"RRRF10K", 3964.0F, 10000.0F, 25.0F, 680, 1600},
    {"Honeywell100K", 3974.0F, 100000.0F, 25.0F, 0, 4700},
    {"Semitec", 4267.0F, 100000.0F, 25.0F, 0, 4700},
    {"HT100K", 3990.0F, 100000.0F, 25.0F, 0, 4700}
};
