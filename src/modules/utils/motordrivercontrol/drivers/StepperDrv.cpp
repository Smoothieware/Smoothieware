#include "StreamOutput.h"

#include "StepperDrv.h"

#include "DRV8711/drv8711.h"
#include "TMC21X/TMC21X.h"
#include "TMC220X/TMC220X.h"
#include "TMC26X/TMC26X.h"

#include <map>

void StepperDrv::set_current(uint16_t currentma){};

void StepperDrv::set_enable(bool enable){};

int StepperDrv::set_microsteps(int number_of_steps) {
	return number_of_steps;
}
int StepperDrv::get_microsteps() { return 0; }

bool StepperDrv::set_options(const StepperDrv::options_t& options){ return false; };

void StepperDrv::set_write_only(bool wo){ this->write_only= wo; };

void StepperDrv::dump_status(StreamOutput *stream){ stream->printf("Not configured.\n"); };

void StepperDrv::get_debug_info(StreamOutput *stream){
	stream->printf("Not supported.\n");
};

bool StepperDrv::set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val){ return false; };
bool StepperDrv::check_alarm(){ return false; };

void StepperDrv::set_chip_type(StepstickParameters::CHIP_TYPE chip) { chip_type= chip; }