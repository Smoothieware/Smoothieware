#include "LcdBase.h"

#include "stdarg.h"
#include "stdio.h"

LcdBase::LcdBase() {}
LcdBase::~LcdBase() {}

int LcdBase::printf(const char* format, ...){
    va_list args;
    va_start(args, format);
    char buffer[80]; // max length for display anyway
    int n= vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    this->write(buffer, n);
    return n;
}
