#include <vector>
#include <string>
#include <iostream>

#include "utils.h"
#include "SerialConsole.h"
#include "gpio.h"

GPIO leds[5] = {
    GPIO(P1_18),
    GPIO(P1_19),
    GPIO(P1_20),
    GPIO(P1_21),
    GPIO(P4_28)
};

#include "easyunit/testharness.h"
#include "easyunit/test.h"

int main( )
{
    Kernel* kernel = new Kernel();

    printf("Starting tests...\n");

    TestRegistry::runAndPrint();

    kernel->serial->printf("Done\n");

    // drop back into DFU upload
    kernel->serial->printf("Entering DFU flash mode...\n");
    system_reset(true);

    for(;;) {}
}
