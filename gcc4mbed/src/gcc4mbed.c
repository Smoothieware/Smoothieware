/* Shim functions used to get mbed based programs to compile and run under GCC */
/* Code originated from startup_LPC17xx.c which contained the following notice */
/****************************************************************************//**
 * @file :    startup_LPC17xx.c
 * @brief : CMSIS Cortex-M3 Core Device Startup File
 * @version : V1.01
 * @date :    4. Feb. 2009
 *
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-Mx
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Updates:
 *  Mod by nio for the .fastcode part
 *  Mod by Sagar G V for &_stack part. Stack Pointer was not getting initialized to RAM0 top..was hanging in the middle
 *  Modified by Sagar G V on Mar 11 2011. added __libc_init_array()
 *  Modfied by Adam Green in 2011 to support mbed.
 ******************************************************************************/
#include "mri.h"


/* Exported constants --------------------------------------------------------*/
extern unsigned long _sidata;       /* start address for the initialization values of the .data section. defined in linker script */
extern unsigned long _sdata;        /* start address for the .data section. defined in linker script */
extern unsigned long _edata;        /* end address for the .data section. defined in linker script */

extern unsigned long _sifastcode;   /* start address for the initialization values of the .fastcode section. defined in linker script */
extern unsigned long _sfastcode;    /* start address for the .fastcode section. defined in linker script */
extern unsigned long _efastcode;    /* end address for the .fastcode section. defined in linker script */

extern unsigned long _sbss;         /* start address for the .bss section. defined in linker script */
extern unsigned long _ebss;         /* end address for the .bss section. defined in linker script */


/* Function prototypes ------------------------------------------------------*/
extern "C" int  main(void);
extern "C" void __libc_init_array(void);
extern "C" void exit(int ErrorCode);
extern "C" void __GCC4MBEDOpenStandardHandles(void);


/* CRT initialization code called from Reset_Handler after it calls SystemInit() */
extern "C" __attribute__ ((section(".mbed_init"))) void __main(void)
{
    unsigned long*  pulDest;
    unsigned long*  pulSrc;
    int             ExitCode;

    /* Copy the data segment initializers from flash to SRAM in ROM mode if needed */
    if (&_sidata != &_sdata)	// only if needed
    {
        pulSrc = &_sidata;
        for(pulDest = &_sdata; pulDest < &_edata; ) 
        {
            *(pulDest++) = *(pulSrc++);
        }
    }

    /* Copy the .fastcode code from ROM to SRAM if needed */
    if (&_sifastcode != &_sfastcode) 
    {
        pulSrc = &_sifastcode;
        for(pulDest = &_sfastcode; pulDest < &_efastcode; ) 
        {
            *(pulDest++) = *(pulSrc++);
        }
    }

    /* Zero fill the bss segment. */
    for(pulDest = &_sbss; pulDest < &_ebss; )
    {
        *(pulDest++) = 0;
    }

    /* Initialize stdin/stdout/stderr file handles. */
    if (!MRI_SEMIHOST_STDIO && !GCC4MBED_DELAYED_STDIO_INIT)
    {
        __GCC4MBEDOpenStandardHandles();
    }
    
    if (MRI_ENABLE)
    {
        __mriInit(MRI_INIT_PARAMETERS);
        if (MRI_BREAK_ON_INIT)
        {
            __debugbreak();
        }
    }

    /* Initialize static constructors. */
     __libc_init_array();

    /* Call the application's entry point. */
    ExitCode = main();
    
    /* Call exit */
    exit(ExitCode);
}

