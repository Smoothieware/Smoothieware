#include "libs/Kernel.h"
#include "PublicData.h"
#include "PublicDataRequest.h"

void PublicData::get_value(uint16_t csa, uint16_t csb, uint16_t csc, void **data) {
	PublicDataRequest pdr(csa, csb, csc);
	this->kernel->call_event(ON_GET_PUBLIC_DATA, &pdr );
	*data= pdr.get_data_ptr();
}

void PublicData::set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data) {

	PublicDataRequest pdr(csa, csb, csc);
	pdr.set_data_ptr(data);
	this->kernel->call_event(ON_SET_PUBLIC_DATA, &pdr );
}
