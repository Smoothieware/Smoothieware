/*Copyright (C) 2011 by Sagar G V, Thejasvi M V

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
/* Updates: 
    Arthur Wolf & Adam Green in 2011 - Updated to work with mbed.
*/

OUTPUT_FORMAT ("elf32-littlearm")
ENTRY(Reset_Handler)
SEARCH_DIR(.)

MEMORY
{
    /* LPC1768 : 512k ROM + 64k SRAM */
    /*------------------------------ */

    /* On-chip ROM is a readable (r), executable region (x) */
    /* On-chip SRAM is a readable (r), writable (w) and */
    /* executable region (x) */

    /* Main ROM region - 512k for LPC1768 */
    /*IROM (rx) : ORIGIN = 0x00000000, LENGTH = 512k */
    IROM (rx) : ORIGIN = 16K, LENGTH = (512k - 16K)

    /* local static RAM - 32k for LPC1768 */
    IRAM0 (rwx) : ORIGIN = 0x10000000, LENGTH = 32k

    /* AHB SRAM - 16k + 16k for LPC1768 */
    IRAM1 (rwx) : ORIGIN = 0x2007C000, LENGTH = 16k
    IRAM2 (rwx) : ORIGIN = 0x20080000, LENGTH = 16k
}

/* SECTION command : Define mapping of input sections */
/* into output sections. */

SECTIONS
{
    /******************************************/
    /* code section */

    /* "normal" code */

    .text :
    {
        KEEP(*(RESET))
        *(.mbed_init)
        *(i.SystemInit)
        . = 0x000002FC;
        KEEP(*(.crp))
        . = 0x00000300;
        *(.text .text.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
        *(.rodata .rodata*)
        *(.gnu.linkonce.r.*)
        . = ALIGN(4);
        KEEP(*(.init))

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(0x4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;


        . = ALIGN(0x4);
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

        /* End Of .text section */
        _etext = .;
        _sifastcode = .;
    } >IROM


    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >IROM
    __exidx_end = .;

    /******************************************/
    /* data section */
    .data :
    {
        _sidata = LOADADDR (.data);
        . = ALIGN(4);
        _sdata = .;
        Image$$RW_IRAM1$$Base = .;

        *(.ARM.__AT_0x10000000)
        *(vtable vtable.*)
        *(.data .data.*)
        *(.gnu.linkonce.d*)

        . = ALIGN(4);
        _edata = . ;
    } >IRAM0 AT>IROM

    /******************************************/
    /* For no-init variables section */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = . ;

        *(.bss .bss.*)
        *(.gnu.linkonce.b*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = . ;
        Image$$RW_IRAM1$$ZI$$Limit = . ;
    } >IRAM0

    /**************************************************/
    /* fastcode - copied at startup & executed in RAM */

    .fastcode :
    {
      _sifastcode = LOADADDR (.fastcode);
      . = ALIGN (4);
      _sfastcode = . ;

      *(.glue_7t) *(.glue_7)
      *(.fastcode)
      
      /* add other modules here ... */

      . = ALIGN (4);
      _efastcode = . ;
    } >IRAM0 AT>IROM

    /******************************************/
    /* For stack section */
    .stackarea (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;

        *(.stackarea .stackarea.*)

        . = ALIGN(8);
        _estack = .;

        . = ALIGN(4);
        _end = . ;
        PROVIDE (end = .);
    } > IRAM0
    _stack = ORIGIN(IRAM0) + LENGTH(IRAM0);

    /******************************************/
    /* Code can explicitly ask for data to be 
       placed in these higher RAM banks where
       they will be left uninitialized. */
    .AHBSRAM0 (NOLOAD):
    {
        Image$$RW_IRAM2$$Base = . ;
        *(AHBSRAM0)
        Image$$RW_IRAM2$$ZI$$Limit = .;
    } > IRAM1

    .AHBSRAM1 (NOLOAD):
    {
        Image$$RW_IRAM3$$Base = . ;
        *(AHBSRAM1)
        Image$$RW_IRAM3$$ZI$$Limit = .;
    } > IRAM2


    /******************************************/
    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    /* .comment       0 : { *(.comment) } */
    /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}
