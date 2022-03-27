#include <stdint.h>
#include <ctype.h>

float parse_float(const char* nptr, char** endptr)
{
	const int kStrtofMaxDigits = 8;
	const char *p = nptr;
	// Skip leading white space, if any. Not necessary
	while (isspace(*p) ) ++p;

	// Get sign, if any.
	bool sign = true;
	if (*p == '-') {
		sign = false; ++p;
	} else if (*p == '+') {
		++p;
	}

	// Get digits before decimal point, if any.
	uint32_t predec;  // to store digits before decimal point
	for (predec = 0; isdigit(*p); ++p) {
		predec = predec * 10UL + static_cast<uint32_t>(*p - '0');
	}
	float value = static_cast<float>(predec);

	// Get digits after decimal point, if any.
	if (*p == '.') {
		uint32_t pow10 = 1;
		uint32_t val2 = 0;
		int digit_cnt = 0;
		++p;
		while (isdigit(*p)) {
			if (digit_cnt < kStrtofMaxDigits) {
				val2 = val2 * 10UL + static_cast<uint32_t>(*p - '0');
				pow10 *= 10UL;
			}  // when kStrtofMaxDigits is read, ignored following digits
			++p;
			++digit_cnt;
		}
		value += static_cast<float>(
		             static_cast<double>(val2) / static_cast<double>(pow10));
	}

#pragma GCC diagnostic ignored "-Wcast-qual"
	if (endptr) *endptr = (char*)p;  // NOLINT(*)
	return sign ? value : - value;
}
