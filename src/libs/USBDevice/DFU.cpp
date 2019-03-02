#include "DFU.h"

// #include <LPC17xx.h>
#include "lpc17xx_wdt.h"

#include <stdio.h>
#include <mri.h>
#include <cstring>

DFU::DFU(USB *u)
{
    usb = u;
    dfu_descriptor = {
        DL_DFU_FUNCTIONAL_DESCRIPTOR,
        DT_DFU_FUNCTIONAL_DESCRIPTOR,
        DFU_BMATTRIBUTES_WILLDETACH | DFU_BMATTRIBUTES_CANDOWNLOAD | DFU_BMATTRIBUTES_CANUPLOAD,
        2000,               // wDetachTimeout
        512,                // wTransferSize
        DFU_VERSION_1_1,    // bcdDFUVersion
    };
    dfu_interface = {
        DL_INTERFACE,
        DT_INTERFACE,
        0,                          // bInterfaceNumber: filled out during addInterface()
        0,                          // bAlternateSetting
        0,                          // bNumEndpoints
        DFU_INTERFACE_CLASS,        // bInterfaceClass
        DFU_INTERFACE_SUBCLASS,     // bInterfaceSubClass
        DFU_INTERFACE_PROTOCOL_RUNTIME, // bInterfaceProtocol
        0,                          // iInterface

        0, 0, 0,                    // dummy padding
        this,                       // callback
    };

    usbdesc_string_l(13) s = usbstring("Smoothie DFU");
    memcpy(&dfu_string, &s, sizeof(dfu_string));

    usb->addInterface(&dfu_interface);
    usb->addDescriptor(&dfu_descriptor);
    dfu_interface.iInterface = usb->addString(&dfu_string);

    prep_for_detach = 0;
}

bool DFU::USBEvent_Request(CONTROL_TRANSFER &control)
{
    printf("Got DFU Control Request: %d length %d\n", control.setup.bRequest, control.setup.wLength);
    switch (control.setup.bRequest)
    {
        case  DFU_DETACH:
        {
//             usb->disconnect();
            prep_for_detach = 128;
            WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
            WDT_Start(250000); // 0.25 seconds
            //             for (;;);
            return true;
        }
        case DFU_GETSTATUS:
        {
//             __debugbreak();
            dfu_status = {
                0,      // status OK
                500,    // bwPollTimeout
                0,      // state appIdle
                0       // iString
            };
            control.direction = DEVICE_TO_HOST;
            control.ptr = (uint8_t *) &dfu_status;
            control.remaining = sizeof(dfu_status);
            return true;
        }
        case DFU_CLRSTATUS:
        {
            return true;
        }
        case DFU_GETSTATE:
        {
            dfu_status.bState = 0; // appIdle
            control.direction = DEVICE_TO_HOST;
            control.ptr = (uint8_t *) &dfu_status.bState;
            control.remaining = 1;
            return true;
        }
        case DFU_ABORT:
        {
            return true;
        }
    }
    return false;
}

bool DFU::USBEvent_RequestComplete(CONTROL_TRANSFER &control, uint8_t *buf, uint32_t length)
{
    return false;
}

void DFU::on_module_loaded()
{
    register_for_event(ON_IDLE);
}

void DFU::on_idle(void* argument)
{
    if (prep_for_detach)
    {
        prep_for_detach--;
        if (prep_for_detach == 0)
        {
            usb->disconnect();
            for (;;);
        }
    }
}
