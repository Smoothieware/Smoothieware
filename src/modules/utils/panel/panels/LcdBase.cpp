#include "LcdBase.h"

LcdBase::LcdBase() {}
LcdBase::~LcdBase() {}


int LcdBase::printf(const char* format, ...){
	va_list args;
	va_start(args, format);
	char buffer[80]; // max length for display anyway
	int n= vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	for(int i=0; i < n; i++){
		this->write(buffer[i]); 
	}
	this->writeDone();
	return 0;
}
