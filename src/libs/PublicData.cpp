#include "libs/Kernel.h"
#include "PublicData.h"
#include "PublicDataRequest.h"

bool PublicData::get_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data) {
    PublicDataRequest pdr(csa, csb, csc);
    // the caller may have created the storage for the returned data so we clear the flag,
    // if it gets set by the callee setting the data ptr that means the data is a pointer to a pointer and is set to a pointer to the returned data
    pdr.set_data_ptr(data, false);
    THEKERNEL->call_event(ON_GET_PUBLIC_DATA, &pdr );
    if(pdr.is_taken() && pdr.has_returned_data()) {
        // the callee set the returned data pointer
        *(void**)data= pdr.get_data_ptr();
    }
    return pdr.is_taken();
}

bool PublicData::set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data) {
    PublicDataRequest pdr(csa, csb, csc);
    pdr.set_data_ptr(data);
    THEKERNEL->call_event(ON_SET_PUBLIC_DATA, &pdr );
    return pdr.is_taken();
}
