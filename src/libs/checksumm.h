/*
    This file is part of Smoothie (http://smoothieware.org/)
    The motion control part is heavily based on Grbl (https://github.com/simen/grbl)

    Smoothie is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
    details.

    You should have received a copy of the GNU General Public License along
    with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
/* Calculates Fletcher Checksum at compile time using recursive macros. */
#ifndef _CHECKSUM_MACRO_H_
#define _CHECKSUM_MACRO_H_

/* There are two compile time checksumming approaches in this file.  One uses
   the C preprocessor and the other uses new C++11 features.  They should both
   do the same thing but my current compiler, GCC 4.7, doesn't always do the
   constant folding for the C++ approach and this causes code bloat.  The C
   preprocessor approach can currently generate the exact same code in
   Checked/Release builds as the old method of pre-calculating checksums by
   hand and pasting them into the code.

   Keeping both versions here: the active C version and the C++ version
   that we should switch to later if newer compilers do the required constant
   folding (maybe there are optimization flags to help here.)
*/
#ifdef CHECKSUM_USE_CPP

#include <type_traits>
#include <stdint.h>
#include <stddef.h>

/* Cool C++11 approach contributed by bgamari on #smoothieware IRC channel.
 * Unfortunately this will have to wait until after we switch to GCC 4.7.
 */
constexpr uint16_t checksum(const char* s, size_t n, size_t i, uint16_t sum1, uint16_t sum2) {
  return (i <= n) ? checksum(s, n, i+1, (sum1 + s[i]) % 255, (sum2 + sum1) % 255) : ((sum2 << 8) | sum1);
}

constexpr uint16_t operator "" _checksum(const char* s, size_t n) {
  return checksum(s, n, 0, 0, 0);
}

#define CHECKSUM(X) std::integral_constant<uint16_t, X##_checksum>::value

#else /* !CHECKSUM_USE_CPP */

/* Adam Green's old and crusty C approach. */
/* Recursively define SUM1, the basic checksum % 255 */
#define SUM1_1(X) ((X)[0] % 255)
#define SUM1_2(X) ((SUM1_1(X) + (X)[1]) % 255)
#define SUM1_3(X) ((SUM1_2(X) + (X)[2]) % 255)
#define SUM1_4(X) ((SUM1_3(X) + (X)[3]) % 255)
#define SUM1_5(X) ((SUM1_4(X) + (X)[4]) % 255)
#define SUM1_6(X) ((SUM1_5(X) + (X)[5]) % 255)
#define SUM1_7(X) ((SUM1_6(X) + (X)[6]) % 255)
#define SUM1_8(X) ((SUM1_7(X) + (X)[7]) % 255)
#define SUM1_9(X) ((SUM1_8(X) + (X)[8]) % 255)
#define SUM1_10(X) ((SUM1_9(X) + (X)[9]) % 255)
#define SUM1_11(X) ((SUM1_10(X) + (X)[10]) % 255)
#define SUM1_12(X) ((SUM1_11(X) + (X)[11]) % 255)
#define SUM1_13(X) ((SUM1_12(X) + (X)[12]) % 255)
#define SUM1_14(X) ((SUM1_13(X) + (X)[13]) % 255)
#define SUM1_15(X) ((SUM1_14(X) + (X)[14]) % 255)
#define SUM1_16(X) ((SUM1_15(X) + (X)[15]) % 255)
#define SUM1_17(X) ((SUM1_16(X) + (X)[16]) % 255)
#define SUM1_18(X) ((SUM1_17(X) + (X)[17]) % 255)
#define SUM1_19(X) ((SUM1_18(X) + (X)[18]) % 255)
#define SUM1_20(X) ((SUM1_19(X) + (X)[19]) % 255)
#define SUM1_21(X) ((SUM1_20(X) + (X)[20]) % 255)
#define SUM1_22(X) ((SUM1_21(X) + (X)[21]) % 255)
#define SUM1_23(X) ((SUM1_22(X) + (X)[22]) % 255)
#define SUM1_24(X) ((SUM1_23(X) + (X)[23]) % 255)
#define SUM1_25(X) ((SUM1_24(X) + (X)[24]) % 255)
#define SUM1_26(X) ((SUM1_25(X) + (X)[25]) % 255)
#define SUM1_27(X) ((SUM1_26(X) + (X)[26]) % 255)
#define SUM1_28(X) ((SUM1_27(X) + (X)[27]) % 255)
#define SUM1_29(X) ((SUM1_28(X) + (X)[28]) % 255)
#define SUM1_30(X) ((SUM1_29(X) + (X)[29]) % 255)
#define SUM1_31(X) ((SUM1_30(X) + (X)[30]) % 255)
#define SUM1_32(X) ((SUM1_31(X) + (X)[31]) % 255)

/* Recursively define SUM2, the sum of SUM1s % 255 */
#define SUM2_1(X) (SUM1_1(X) % 255)
#define SUM2_2(X) ((SUM2_1(X) + SUM1_2(X)) % 255)
#define SUM2_3(X) ((SUM2_2(X) + SUM1_3(X)) % 255)
#define SUM2_4(X) ((SUM2_3(X) + SUM1_4(X)) % 255)
#define SUM2_5(X) ((SUM2_4(X) + SUM1_5(X)) % 255)
#define SUM2_6(X) ((SUM2_5(X) + SUM1_6(X)) % 255)
#define SUM2_7(X) ((SUM2_6(X) + SUM1_7(X)) % 255)
#define SUM2_8(X) ((SUM2_7(X) + SUM1_8(X)) % 255)
#define SUM2_9(X) ((SUM2_8(X) + SUM1_9(X)) % 255)
#define SUM2_10(X) ((SUM2_9(X) + SUM1_10(X)) % 255)
#define SUM2_11(X) ((SUM2_10(X) + SUM1_11(X)) % 255)
#define SUM2_12(X) ((SUM2_11(X) + SUM1_12(X)) % 255)
#define SUM2_13(X) ((SUM2_12(X) + SUM1_13(X)) % 255)
#define SUM2_14(X) ((SUM2_13(X) + SUM1_14(X)) % 255)
#define SUM2_15(X) ((SUM2_14(X) + SUM1_15(X)) % 255)
#define SUM2_16(X) ((SUM2_15(X) + SUM1_16(X)) % 255)
#define SUM2_17(X) ((SUM2_16(X) + SUM1_17(X)) % 255)
#define SUM2_18(X) ((SUM2_17(X) + SUM1_18(X)) % 255)
#define SUM2_19(X) ((SUM2_18(X) + SUM1_19(X)) % 255)
#define SUM2_20(X) ((SUM2_19(X) + SUM1_20(X)) % 255)
#define SUM2_21(X) ((SUM2_20(X) + SUM1_21(X)) % 255)
#define SUM2_22(X) ((SUM2_21(X) + SUM1_22(X)) % 255)
#define SUM2_23(X) ((SUM2_22(X) + SUM1_23(X)) % 255)
#define SUM2_24(X) ((SUM2_23(X) + SUM1_24(X)) % 255)
#define SUM2_25(X) ((SUM2_24(X) + SUM1_25(X)) % 255)
#define SUM2_26(X) ((SUM2_25(X) + SUM1_26(X)) % 255)
#define SUM2_27(X) ((SUM2_26(X) + SUM1_27(X)) % 255)
#define SUM2_28(X) ((SUM2_27(X) + SUM1_28(X)) % 255)
#define SUM2_29(X) ((SUM2_28(X) + SUM1_29(X)) % 255)
#define SUM2_30(X) ((SUM2_29(X) + SUM1_30(X)) % 255)
#define SUM2_31(X) ((SUM2_30(X) + SUM1_31(X)) % 255)
#define SUM2_32(X) ((SUM2_31(X) + SUM1_32(X)) % 255)

/* Define overall checksum as 16-bit combination of SUM1 in lower 8-bits and SUM2 in upper 8-bits. */
#define CHECKSUM_(SUM1,SUM2) (SUM1) | (SUM2 << 8)
#define CHECKSUM_1(X) CHECKSUM_(SUM1_1(X),SUM2_1(X))
#define CHECKSUM_2(X) CHECKSUM_(SUM1_2(X),SUM2_2(X))
#define CHECKSUM_3(X) CHECKSUM_(SUM1_3(X),SUM2_3(X))
#define CHECKSUM_4(X) CHECKSUM_(SUM1_4(X),SUM2_4(X))
#define CHECKSUM_5(X) CHECKSUM_(SUM1_5(X),SUM2_5(X))
#define CHECKSUM_6(X) CHECKSUM_(SUM1_6(X),SUM2_6(X))
#define CHECKSUM_7(X) CHECKSUM_(SUM1_7(X),SUM2_7(X))
#define CHECKSUM_8(X) CHECKSUM_(SUM1_8(X),SUM2_8(X))
#define CHECKSUM_9(X) CHECKSUM_(SUM1_9(X),SUM2_9(X))
#define CHECKSUM_10(X) CHECKSUM_(SUM1_10(X),SUM2_10(X))
#define CHECKSUM_11(X) CHECKSUM_(SUM1_11(X),SUM2_11(X))
#define CHECKSUM_12(X) CHECKSUM_(SUM1_12(X),SUM2_12(X))
#define CHECKSUM_13(X) CHECKSUM_(SUM1_13(X),SUM2_13(X))
#define CHECKSUM_14(X) CHECKSUM_(SUM1_14(X),SUM2_14(X))
#define CHECKSUM_15(X) CHECKSUM_(SUM1_15(X),SUM2_15(X))
#define CHECKSUM_16(X) CHECKSUM_(SUM1_16(X),SUM2_16(X))
#define CHECKSUM_17(X) CHECKSUM_(SUM1_17(X),SUM2_17(X))
#define CHECKSUM_18(X) CHECKSUM_(SUM1_18(X),SUM2_18(X))
#define CHECKSUM_19(X) CHECKSUM_(SUM1_19(X),SUM2_19(X))
#define CHECKSUM_20(X) CHECKSUM_(SUM1_20(X),SUM2_20(X))
#define CHECKSUM_21(X) CHECKSUM_(SUM1_21(X),SUM2_21(X))
#define CHECKSUM_22(X) CHECKSUM_(SUM1_22(X),SUM2_22(X))
#define CHECKSUM_23(X) CHECKSUM_(SUM1_23(X),SUM2_23(X))
#define CHECKSUM_24(X) CHECKSUM_(SUM1_24(X),SUM2_24(X))
#define CHECKSUM_25(X) CHECKSUM_(SUM1_25(X),SUM2_25(X))
#define CHECKSUM_26(X) CHECKSUM_(SUM1_26(X),SUM2_26(X))
#define CHECKSUM_27(X) CHECKSUM_(SUM1_27(X),SUM2_27(X))
#define CHECKSUM_28(X) CHECKSUM_(SUM1_28(X),SUM2_28(X))
#define CHECKSUM_29(X) CHECKSUM_(SUM1_29(X),SUM2_29(X))
#define CHECKSUM_30(X) CHECKSUM_(SUM1_30(X),SUM2_30(X))
#define CHECKSUM_31(X) CHECKSUM_(SUM1_31(X),SUM2_31(X))
#define CHECKSUM_32(X) CHECKSUM_(SUM1_32(X),SUM2_32(X))

#ifdef DEBUG

#include <utils.h>
#define CHECKSUM(X) get_checksum(X)

#else /* !DEBUG */

#define CHECKSUM(X) (sizeof(X) == 0 ? 0 : \
                     sizeof(X) == 1 ? 0 : \
                     sizeof(X) == 2 ? CHECKSUM_1(X) : \
                     sizeof(X) == 3 ? CHECKSUM_2(X) : \
                     sizeof(X) == 4 ? CHECKSUM_3(X) : \
                     sizeof(X) == 5 ? CHECKSUM_4(X) : \
                     sizeof(X) == 6 ? CHECKSUM_5(X) : \
                     sizeof(X) == 7 ? CHECKSUM_6(X) : \
                     sizeof(X) == 8 ? CHECKSUM_7(X) : \
                     sizeof(X) == 9 ? CHECKSUM_8(X) : \
                     sizeof(X) == 10 ? CHECKSUM_9(X) : \
                     sizeof(X) == 11 ? CHECKSUM_10(X) : \
                     sizeof(X) == 12 ? CHECKSUM_11(X) : \
                     sizeof(X) == 13 ? CHECKSUM_12(X) : \
                     sizeof(X) == 14 ? CHECKSUM_13(X) : \
                     sizeof(X) == 15 ? CHECKSUM_14(X) : \
                     sizeof(X) == 16 ? CHECKSUM_15(X) : \
                     sizeof(X) == 17 ? CHECKSUM_16(X) : \
                     sizeof(X) == 18 ? CHECKSUM_17(X) : \
                     sizeof(X) == 19 ? CHECKSUM_18(X) : \
                     sizeof(X) == 20 ? CHECKSUM_19(X) : \
                     sizeof(X) == 21 ? CHECKSUM_20(X) : \
                     sizeof(X) == 22 ? CHECKSUM_21(X) : \
                     sizeof(X) == 23 ? CHECKSUM_22(X) : \
                     sizeof(X) == 24 ? CHECKSUM_23(X) : \
                     sizeof(X) == 25 ? CHECKSUM_24(X) : \
                     sizeof(X) == 26 ? CHECKSUM_25(X) : \
                     sizeof(X) == 27 ? CHECKSUM_26(X) : \
                     sizeof(X) == 28 ? CHECKSUM_27(X) : \
                     sizeof(X) == 29 ? CHECKSUM_28(X) : \
                     sizeof(X) == 30 ? CHECKSUM_29(X) : \
                     sizeof(X) == 31 ? CHECKSUM_30(X) : \
                     sizeof(X) == 32 ? CHECKSUM_31(X) : \
                     sizeof(X) == 33 ? CHECKSUM_32(X) : \
                     0xFFFF)
#endif /* DEBUG */
#endif /* CHECKSUM_USE_CPP */

#endif /* _CHECKSUM_MACRO_H_ */
