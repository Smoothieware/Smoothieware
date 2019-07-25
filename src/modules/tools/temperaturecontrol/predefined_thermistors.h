// Beta algorithm
float calc_beta(thermistor_t *t, float r)
{
	return (1.0F / ((1.0F / (t->t0 + 273.15F)) + (logf(r / t->r0) / t->beta))) - 273.15F;
}

// Steinhart-Hart algorithm
float calc_sh(thermistor_t *t, float r)
{
	float l = logf(r);
	return (1.0F / (t->c1 + t->c2 * l + t->c3 * powf(l, 3))) - 273.15F;
}

// used for Pt1000 sensor
// See more: http://forum.smoothieware.org/forum/t-1086570
float calc_ptc(thermistor_t *t, float r)
{
	return t->cc0 + (t->cc1 * r) + (t->cc2 * r * r);
}

float calc_ptc2(thermistor_t *t, float r)
{
	return (sqrt(t->cc0 * t->cc0 - 4 * t->cc1 * (1 - r / t->cc2)) - t->cc0) / (2 * t->cc1);
}

// Use one of the following scripts to calculate the coefficients:
// - http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// - https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
// - Pt1000 coefficients from: http://forum.smoothieware.org/forum/t-1086570
static const thermistor_t predefined_thermistors[] {
    // name,            r1,		r2,   	c1,                    		c2,                     c3
    {"EPCOS100K",       0, 		4700, 	0.000722378300319346F, 		0.000216301852054578F,  9.2641025635702e-08F,	calc_sh 	}, // B57540G0104F000
    {"Vishay100K",      0, 		4700, 	0.0007022607370F,      		0.0002209155484F,       7.101626461e-08F,	 	calc_sh		}, // NTCS0603E3104FXT
    {"Honeywell100K",   0, 		4700, 	0.000596153185928425F, 		0.000231333192738335F,  6.19534004306738e-08F,	calc_sh		}, // 135-104LAG-J01
    {"Semitec",         0, 		4700, 	0.000811290160145459F, 		0.000211355789144265F,  7.17614730463848e-08F,	calc_sh		}, // 104GT-2
    {"Honeywell-QAD",   0,		4700, 	0.000827339299500986F, 		0.000208786427208899F,  8.05595282332277e-08F,	calc_sh		}, // 135-104QAD-J01
    {"Semitec-104NT4",  0,		4700, 	0.000797110609710217F, 		0.000213433144381270F,  6.5338987554e-08F,		calc_sh		}, // 104NT-4R025H42G

    // name,            r1,  	r2,   	beta,    					r0,        				t0
//	{"EPCOS100K",       0,   	4700,	4066.0F, 					100000.0F, 				25.0F,					calc_beta	}, // B57540G0104F000
    {"RRRF100K",        0, 		4700, 	3960.0F, 					100000.0F, 				25.0F, 					calc_beta	},
    {"RRRF10K",         680,	1600, 	3964.0F, 					10000.0F,  				25.0F,					calc_beta	},
//	{"Honeywell100K",   0, 		4700, 	3974.0F, 					100000.0F, 				25.0F,					calc_beta	}, // 135-104LAG-J01
//	{"Semitec",         0, 		4700, 	4267.0F, 					100000.0F, 				25.0F,					calc_beta	}, // 104GT-2
    {"HT100K",          0, 		4700, 	3990.0F, 					100000.0F, 				25.0F,					calc_beta	},

	// name,			r1,		r2,		cc0,						cc1,					cc2
//	{"Pt1000",  		0,   	4700, 	-2.457445131972171e+02F, 	2.356226766421429e-01F,	1.011383918573980e-05F,	calc_ptc	},
//	{"Pt100",  			0,   	4700, 	-2.457445131972171e+02F, 	2.356226766421429,		1.011383918573980e-03F,	calc_ptc	},
	// name,			r1,		r2,		a,							b,						r0
    {"Pt1000",  		0,   	4700, 	3.9083e-3F, 				-5.775e-7F,				1000,					calc_ptc2	},
    {"Pt100",  			0,   	4700, 	3.9083e-3F, 				-5.775e-7F,				100,					calc_ptc2	},
};
