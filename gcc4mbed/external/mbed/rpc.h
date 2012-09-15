/* mbed Microcontroller Library - RPC
 * Copyright (c) 2008-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_RPC_H
#define MBED_RPC_H

/* Section rpc
 *  Helpers for rpc handling.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "Base.h"

#include "PinNames.h"
#include <stdint.h>

namespace mbed {

/* Function parse_arg
 *  Parses and returns a value from a string.
 *
 * Variable
 *  arg - The string to pase
 *  next - If not NULL a pointer to after the last 
 *    character parsed is written here
 */
template<typename T> T parse_arg(const char *arg, const char **next);

inline char parse_char(const char *arg, const char **next) {
    char c = *arg++;
    if(c == '\\') {
        c = *arg++;
        switch(c) {
        case 'a': c = '\a'; break;
        case 'b': c = '\b'; break;
        case 't': c = '\t'; break;
        case 'n': c = '\n'; break;
        case 'v': c = '\v'; break;
        case 'f': c = '\f'; break;
        case 'r': c = '\r'; break;
        case 'x': 
            {
                /* two-character hexadecimal */
                char buf[3];
                buf[0] = *arg++;
                buf[1] = *arg++;
                buf[2] = 0;
                c = strtol(buf, NULL, 16); 
            }
            break;
        default: 
            if(isdigit(c)) {
                /* three-character octal */
                char buf[4];
                buf[0] = c;
                buf[1] = *arg++;
                buf[2] = *arg++;
                buf[3] = 0;
                c = strtol(buf, NULL, 8); 
            }
            break;
        }
    }
    *next = arg;
    return c;
}

/* signed integer types */

template<> inline int parse_arg<int>(const char *arg, const char **next) {
    if(arg[0] == '\'') {
        char c = parse_char(arg+1, &arg);
        if(next != NULL) *next = arg+1;
        return c;
    } else {
        return strtol(arg, const_cast<char**>(next), 0);        
    }
}

template<> inline char parse_arg<char>(const char *arg, const char **next) {
    return parse_arg<int>(arg,next);
}

template<> inline short int parse_arg<short int>(const char *arg, const char **next) {
    return parse_arg<int>(arg,next);
}

template<> inline long int parse_arg<long int>(const char *arg, const char **next) {
    return parse_arg<int>(arg,next);
}

template<> inline long long parse_arg<long long>(const char *arg, const char **next) {
    return strtoll(arg, const_cast<char**>(next), 0);
}

/* unsigned integer types */

template<> inline unsigned int parse_arg<unsigned int>(const char *arg, const char **next) {
    if(arg[0] == '\'') {
        char c = parse_char(arg+1, &arg);
        if(next != NULL) *next = arg+1;
        return c;
    } else {
        return strtoul(arg, const_cast<char**>(next), 0);        
    }
}

template<> inline unsigned char parse_arg<unsigned char>(const char *arg, const char **next) {
    return parse_arg<unsigned int>(arg,next);
}

template<> inline unsigned short int parse_arg<unsigned short int>(const char *arg, const char **next) {
    return parse_arg<unsigned int>(arg,next);
}

template<> inline unsigned long int parse_arg<unsigned long int>(const char *arg, const char **next) {
    return parse_arg<unsigned int>(arg,next);
}

template<> inline unsigned long long parse_arg<unsigned long long>(const char *arg, const char **next) {
    return strtoull(arg, const_cast<char**>(next), 0);
}

/* floating types */

template<> inline float parse_arg<float>(const char *arg, const char **next) {
#if !defined(__ARMCC_VERSION) || __ARMCC_VERSION >= 410000
    return strtof(arg,const_cast<char**>(next));
#elif __ARMCC_VERSION >= 310000
    /* bug in header means no using declaration for strtof */
    return std::strtof(arg,const_cast<char**>(next));    
#else
    /* strtof not supported */
    return strtod(arg,const_cast<char**>(next));
#endif
}

template<> inline double parse_arg<double>(const char *arg, const char **next) {
    return strtod(arg,const_cast<char**>(next));
}

template<> inline long double parse_arg<long double>(const char *arg, const char **next) {
    return strtod(arg,const_cast<char**>(next));
}

/* string */

template<> inline char *parse_arg<char*>(const char *arg, const char **next) {
    const char *ptr = arg;
    char *res = NULL;
    if(*arg == '"') {
        /* quoted string */
        ptr = ++arg;
        int len = 0;
        /* find the end (and length) of the quoted string */
        for(char c = *ptr; c != 0 && c != '"'; c = *++ptr) {
            len++;
            if(c == '\\') {
                ptr++;
            }
        }
        /* copy the quoted string, and unescape characters */
        if(len != 0) {
            res = new char[len+1];
            char *resptr = res;
            while(arg != ptr) {
                *resptr++ = parse_char(arg, &arg);
            }
            *resptr = 0;
        }
    } else {
        /* unquoted string */
        while(isalnum(*ptr) || *ptr=='_') {
            ptr++;
        }
        int len = ptr-arg;
        if(len!=0) {
            res = new char[len+1];
            memcpy(res, arg, len);
            res[len] = 0;
        }
    }

    if(next != NULL) {
        *next = ptr;
    }
    return res;
}

template<> inline const char *parse_arg<const char*>(const char *arg, const char **next) {
    return parse_arg<char*>(arg,next);
}

/* Pins */


inline PinName parse_pins(const char *str) {
    const PinName pin_names[] = {p5, p6, p7, p8, p9, p10, p11, p12, p13, p14
                                , p15, p16, p17, p18, p19, p20, p21, p22, p23
                                , p24, p25, p26, p27, p28, p29, p30};

    if(str[0] == 'P') { // Pn_n
        uint32_t port = str[1] - '0';
        uint32_t pin = str[3] - '0'; // Pn_n
        uint32_t pin2 = str[4] - '0'; // Pn_nn
        if(pin2 <= 9) {
            pin = pin * 10 + pin2;
        }
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
        return (PinName)(LPC_GPIO0_BASE + port * 32 + pin);
#elif defined(TARGET_LPC11U24)
        return (PinName)(port * 32 + pin);
#endif
    } else if(str[0] == 'p') {  // pn
        uint32_t pin = str[1] - '0'; // pn
        uint32_t pin2 = str[2] - '0'; // pnn
        if(pin2 <= 9) {
                  pin = pin * 10 + pin2;
        }
        if(pin < 5 || pin > 30) {
	          return NC;
        }
        return pin_names[pin - 5];
    } else if(str[0] == 'L') {  // LEDn
        switch(str[3]) {
            case '1' : return LED1;
            case '2' : return LED2;
            case '3' : return LED3;
            case '4' : return LED4;
        }
    } else if(str[0] == 'U') {  // USB?X
        switch(str[3]) {
            case 'T' : return USBTX;
            case 'R' : return USBRX;
        }
    }
    return NC;
}

template<> inline PinName parse_arg<PinName>(const char *arg, const char **next) {
    const char *ptr = arg;
    PinName pinname = NC;
    while(isalnum(*ptr) || *ptr=='_') {
        ptr++;
    }
    int len = ptr-arg;
    if(len!=0) {
        pinname = parse_pins(arg);
    
    }
    if(next != NULL) {
        *next = ptr;
    }
    return pinname;
}


/* Function write_result
 *  Writes a value in to a result string in an appropriate manner
 *
 * Variable
 *  val - The value to write
 *  result - A pointer to the array to write the value into
 */
template<typename T> void write_result(T val, char *result);

/* signed integer types */

template<> inline void write_result<char>(char val, char *result) {
    result[0] = val;
    result[1] = '\0';
}

template<> inline void write_result<short int>(short int val, char *result) {
    sprintf(result, "%hi", val); 
}

template<> inline void write_result<int>(int val, char *result) {
    sprintf(result, "%i", val); 
}

template<> inline void write_result<long int>(long int val, char *result) {
    sprintf(result, "%li", val); 
}

template<> inline void write_result<long long int>(long long int val, char *result) {
    sprintf(result, "%lli", val); 
}

/* unsigned integer types */

template<> inline void write_result<unsigned char>(unsigned char val, char *result) {
    result[0] = val;
    result[1] = '\0';
}

template<> inline void write_result<unsigned short int>(unsigned short int val, char *result) {
    sprintf(result, "%hu", val); 
}

template<> inline void write_result<unsigned int>(unsigned int val, char *result) {
    sprintf(result, "%u", val); 
}

template<> inline void write_result<unsigned long int>(unsigned long int val, char *result) {
    sprintf(result, "%lu", val); 
}

template<> inline void write_result<unsigned long long int>(unsigned long long int val, char *result) {
    sprintf(result, "%llu", val); 
}

/* floating types */

template<> inline void write_result<float>(float val, char *result) {
    sprintf(result, "%.17g", val); 
}

template<> inline void write_result<double>(double val, char *result) {
    sprintf(result, "%.17g", val); 
}

template<> inline void write_result<long double>(long double val, char *result) {
    sprintf(result, "%.17Lg", val); 
}


/* string */

template<> inline void write_result<char*>(char *val, char *result) {
    if(val==NULL) {
        result[0] = 0;
    } else {
        strcpy(result, val);
    }
}

template<> inline void write_result<const char*>(const char *val, char *result) {
    if(val==NULL) {
        result[0] = 0;
    } else {
        strcpy(result, val);
    }
}


inline const char *next_arg(const char* next) {
    while(*next == ' ') next++;
    if(*next == ',' || *next == '?') next++;
    while(*next == ' ') next++;
    return next;
}


/* Function rpc_method_caller
 */
template<class T, void (T::*member)(const char *,char *)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {
    (static_cast<T*>(this_ptr)->*member)(arguments,result); 
}


/* Function rpc_method_caller
 */
template<class T, void (T::*member)()> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) { 
    (static_cast<T*>(this_ptr)->*member)(); 
    if(result != NULL) {
        result[0] = '\0';
    }
}


/* Function rpc_method_caller
 */
template<class T, typename A1, void (T::*member)(A1)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),NULL);

    (static_cast<T*>(this_ptr)->*member)(arg1); 
    if(result != NULL) {
        result[0] = '\0';
    }
}


/* Function rpc_method_caller
 */
template<class T, typename A1, typename A2, void (T::*member)(A1,A2)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),NULL);

    (static_cast<T*>(this_ptr)->*member)(arg1,arg2);
    if(result != NULL) {
        result[0] = '\0';
    }
}


/* Function rpc_method_caller
 */
template<class T, typename A1, typename A2, typename A3, void (T::*member)(A1,A2,A3)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),&next);
    A3 arg3 = parse_arg<A3>(next_arg(next),NULL);

    (static_cast<T*>(this_ptr)->*member)(arg1,arg2,arg3);
    if(result != NULL) {
        result[0] = '\0';
    }
}


/* Function rpc_method_caller
 */
template<typename R, class T, R (T::*member)()> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) { 
    R res = (static_cast<T*>(this_ptr)->*member)();
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_method_caller
 */
template<typename R, class T, typename A1, R (T::*member)(A1)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),NULL);

    R res = (static_cast<T*>(this_ptr)->*member)(arg1);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_method_caller
 */
template<typename R, class T, typename A1, typename A2, R (T::*member)(A1,A2)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),NULL);

    R res = (static_cast<T*>(this_ptr)->*member)(arg1,arg2);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_method_caller
 */
template<typename R, class T, typename A1, typename A2, typename A3, R (T::*member)(A1,A2,A3)> 
void rpc_method_caller(Base *this_ptr, const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),&next);
    A3 arg3 = parse_arg<A3>(next_arg(next),NULL);

    R res = (static_cast<T*>(this_ptr)->*member)(arg1,arg2,arg3);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_function caller
 */
template<typename R, R (*func)()>
void rpc_function_caller(const char *arguments, char *result) {
    R res = (*func)();
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_function caller
 */
template<typename R, typename A1, R (*func)(A1)>
void rpc_function_caller(const char *arguments, char *result) {
    A1 arg1 = parse_arg<A1>(next_arg(arguments),NULL);
    R res = (*func)(arg1);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_function caller
 */
template<typename R, typename A1, typename A2, R (*func)(A1,A2)>
void rpc_function_caller(const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),NULL);

    R res = (*func)(arg1,arg2);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_function caller
 */
template<typename R, typename A1, typename A2, typename A3, R (*func)(A1,A2,A3)>
void rpc_function_caller(const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),&next);
    A3 arg3 = parse_arg<A3>(next_arg(next),NULL);

    R res = (*func)(arg1,arg2,arg3);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


/* Function rpc_function caller
 */
template<typename R, typename A1, typename A2, typename A3, typename A4, R (*func)(A1,A2,A3,A4)>
void rpc_function_caller(const char *arguments, char *result) {

    const char *next = arguments;
    A1 arg1 = parse_arg<A1>(next_arg(next),&next);
    A2 arg2 = parse_arg<A2>(next_arg(next),&next);
    A3 arg3 = parse_arg<A3>(next_arg(next),&next);
    A4 arg4 = parse_arg<A4>(next_arg(next),NULL);

    R res = (*func)(arg1,arg2,arg3,arg4);
    if(result != NULL) {
        write_result<R>(res, result);
    }
}


struct rpc_method { 
    const char *name;
    typedef void (*caller_t)(Base*, const char*, char*);
    typedef const struct rpc_method *(*super_t)(Base*);
    union {
        caller_t caller;
        super_t super;
    };
};

template<class C>
const struct rpc_method *rpc_super(Base *this_ptr) {
    return static_cast<C*>(this_ptr)->C::get_rpc_methods();
}

#define RPC_METHOD_END { NULL, NULL }
#define RPC_METHOD_SUPER(C) { NULL, (rpc_method::caller_t)(rpc_method::super_t)rpc_super<C> }

/* Function rpc
 *  Parse a string describing a call and then do it
 *
 * Variables
 *  call - A pointer to a string describing the call, which has
 *    the form /object/method arg ... argn. Arguments are
 *    delimited by space characters, and the string is terminated
 *    by a null character.
 *  result - A pointer to an array to write the result into.
 */
bool rpc(const char *buf, char *result = 0);


} // namespace mbed

#endif
