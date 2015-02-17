typedef struct {
        const char *name;
        int r1;
        int r2;
        float beta, r0, t0;
} const thermistor_beta_table_t;

static const thermistor_beta_table_t predefined_thermistors_beta[] {
    // name,            r1,  r2,   beta,    r0,        t0
    {"EPCOS100K",       0,   4700, 4066.0F, 100000.0F, 25.0F}, // B57540G0104F000
    {"RRRF100K",        0,   4700, 3960.0F, 100000.0F, 25.0F},
    {"RRRF10K",         680, 1600, 3964.0F, 10000.0F,  25.0F},
    {"Honeywell100K",   0,   4700, 3974.0F, 100000.0F, 25.0F}, // 135-104LAG-J01
    {"Semitec",         0,   4700, 4267.0F, 100000.0F, 25.0F}, // 104GT-2
    {"HT100K",          0,   4700, 3990.0F, 100000.0F, 25.0F}
};

typedef struct {
        const char *name;
        int r1;
        int r2;
        float c1, c2, c3;
} const thermistor_table_t;

// Use one of the following scripts to calcuate the coefficients:
// - http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// - https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
static const thermistor_table_t predefined_thermistors[] {
    // name,            r1,  r2,   c1,                    c2,                     c3
    {"EPCOS100K",       0,   4700, 0.000722378300319346F, 0.000216301852054578F,  9.2641025635702e-08F},  // B57540G0104F000
    {"Vishay100K",      0,   4700, 0.0007022607370F,      0.0002209155484F,       7.101626461e-08F    },  // NTCS0603E3104FXT
    {"Honeywell100K",   0,   4700, 0.000596153185928425F, 0.000231333192738335F,  6.19534004306738e-08F}, // 135-104LAG-J01
    {"Semitec",         0,   4700, 0.000811290160145459F, 0.000211355789144265F,  7.17614730463848e-08F}, // 104GT-2
    {"Honeywell-QAD",   0,   4700, 0.000827339299500986F, 0.000208786427208899F,  8.05595282332277e-08F}  // 135-104QAD-J01
};
