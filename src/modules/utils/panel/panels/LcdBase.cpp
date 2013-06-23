#include "LcdBase.h"

LcdBase::LcdBase() {}
LcdBase::~LcdBase() {}


int LcdBase::printf(const std::string format, ...){
	wait_us(10); 
	va_list args;
	va_start(args, format);
	int size = format.size() * 2;
	char* buffer = new char[size];
	while (vsprintf(buffer, format.c_str(), args) < 0){
		delete[] buffer;
		size *= 2;
		buffer = new char[size];
	}
	string message = std::string(buffer);
	va_end(args);
	for(unsigned int i=0; i < message.size(); i++){
		this->write(message.at(i)); 
	}
	delete[] buffer;
	return 0;
}
