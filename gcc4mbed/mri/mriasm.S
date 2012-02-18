/* Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Implementation of Assembly Language routines to be used by MRI debug monitor. */
    .text
    .code 16
    .syntax unified
    
    
    /* NOTE: These symbols must match the equivalent #defines in mri.c */
    .equ MRI_FLAGS_ACTIVE_DEBUG, 1
    .equ MRI_FLAGS_FAULT_DURING_DEBUG, 2
    

    .global MriDisableMbed
    /* extern "C" int MriDisableMbed(void);
       Attempts to disable the debug interface on the mbed device.
    
       Returns -1 if the JTAG debugger isn't the mbed interface chip.
       
       NOTE: This code was taken from a beta mbed library available at:
             http://mbed.org/projects/libraries-testing/svn/beta
    */
MriDisableMbed:
    sub     sp, #8
    add     r1, sp, #4
    movw    r0, #261
    bkpt	0x00ab
    cmp     r0, #0
    ite     eq
    ldreq   r0, [sp, #4]
    movne.w r0, #-1
    add     sp, #8
    bx      lr
    
    
    .global MriGetIPSR
    /* extern "C" unsigned int MriGetIPSR(void)
       Returns the current value of the IPSR register.
    */
MriGetIPSR:
    mrs     r0, ipsr
    bx      lr
    
    
    /* Handles exception invokations for MRI.  Its main job is to store the interrupted task's context, call into
       the C based MriDebugException, and then restore the task's context (which may have been modified by the
       debugger.
       
       Layout of the registers which are pushed on the stack automatically by Cortex-M3 curing exception execution:
            SP      Value in MSP or PSP task was interrupted.
        PSR SP - 4
        PC  SP - 8
        LR  SP - 12
        R12 SP - 16
        R3  SP - 20
        R2  SP - 24
        R1  SP - 28
        R0  SP - 32  Value in MSP or PSP when handler is started.
        
       Layout of the SContext record used by gdb for 'g' and 'G' commands.
        typedef struct
        {
            unsigned int    R0;
            unsigned int    R1;
            unsigned int    R2;
            unsigned int    R3;
            unsigned int    R4;
            unsigned int    R5;
            unsigned int    R6;
            unsigned int    R7;
            unsigned int    R8;
            unsigned int    R9;
            unsigned int    R10;
            unsigned int    R11;
            unsigned int    R12;
            unsigned int    SP;
            unsigned int    LR;
            unsigned int    PC;
            unsigned int    Floats[8 * 3];
            unsigned int    FPS;
            unsigned int    CPSR;
        } SContext;
    */
MriExceptionHandler:
    /**** Task may have been using PSP or MSP so retrieve the correct one into R1. */
    tst     lr, #0x4
    ite     eq
    mrseq   r1, msp
    mrsne   r1, psp
    
    /**** Detect fault encountered during debug and in such cases, only set flag and skip overwrite of existing context. */
    ldr     r0, =g_MriFlags
    ldr     r2, [r0]
    /* If already debugging, then set flag and jump to code which will advance PC before resuming execution. */
    tst     r2, #MRI_FLAGS_ACTIVE_DEBUG
    ittt    ne
    orrne   r2, #MRI_FLAGS_FAULT_DURING_DEBUG
    strne   r2, [r0]
    bne     __MriAdvancePCAndReturn
    
    /**** Copy R0-R3, R12, LR, PC, and PSR from running task's stack and place in MRI context structure. */
    ldr     r0, =g_MriContext
    /* Now that R1 contains the SP, we can push away the current LR (and R12 to just keep 8-byte alignment.) */
    push    {r12,lr}
    /* R0 is now destination pointer to beginning of context record.
       R1 is now source pointer to end of auto stacking area. */
    /* Copy R0 - R3 from stacked area to context. */
    ldmia   r1!, {r2,r3,r12,lr}
    stmia   r0!, {r2,r3,r12,lr}
    /* Store R4 - R11 to context. */
    stmia   r0!, {r4-r11}
    /* Load R12, LR, PC, and PSR from stacked area. */
    ldmia   r1!, {r2-r5}
    /* Store R12 to context. */
    str     r2, [r0], #8      /* Skip over SP which was already saved above. */
    /* Store LR & PC to context. */
    stmia   r0!, {r3, r4}
    /* Store PSR to context, skipping floating point registers and status. */
    str     r5, [r0, #8*12+4]
    /* R1 will now be pointing to the task's SP at interrupt time unless the stack was 8-byte aligned. */
    tst     r5, #0x200
    it      ne
    addne   r1, r1, #4
    /* Jump back past the PC and LR that were placed in context to store the potentially updated SP. */
    str     r1, [r0, #-12]

    /**** Run the C routine which allows for debugging of exceptions. */
    bl      MriDebugException

    /**** Restore the tasks' registers which may have been modified by debugger. */
    /* Point context source pointer to location of PSR data. */
    ldr     r0, =g_MriContext+16*4+8*12+4
    /* Retrieve value of task's SP at time of interrupt.  May have been changed by debug client. */
    ldr     r1, [r0, #-(4+8*12+3*4)]
    /* Does it need to be 8-byte aligned? Fetch PSR, advance down past floating point registers, and check bit 9. */
    ldr     r2, [r0], #-(4+8*12)
    tst     r2, #0x200
    it      ne
    subne   r1, r1, #4
    /* Store PSR on task stack. */
    str     r2, [r1, #-4]!
    /* Copy PC and LR from context to stacking area. */
    ldmdb   r0!, {r2,r3}
    stmdb   r1!, {r2,r3}
    /* Copy r12 from context to stack area.  Skip over SP in context as it was already handled above. */
    ldr     r12, [r0, #-8]!
    str     r12, [r1, #-4]!
    /* Load R4-R11 from context. */
    ldmdb   r0!, {r4-r11}
    /* Copy R0 - R3 from context to stack area. */
    ldmdb   r0!, {r2,r3,r12,lr}
    stmdb   r1!, {r2,r3,r12,lr}
    /* Task may have been using PSP or MSP so store the new SP (after auto stacking) in appropriate register. */
    pop     {r12,lr}
    tst     lr, #0x4
    ite     eq
    msreq   msp,r1
    msrne   psp,r1

    /**** Make sure that any modifications to the instruction stream by data writes are seen. */
    dsb
    isb
    
    /**** Return to task */
    bx      lr


    .type __MriAdvancePCAndReturn, function
    /* Branched to from MriException handler if the debugger is already active.  Getting an exception in such a
       scenario typically means that a memory fault occurred while attempting to access an invalid memory address so
       it should be flagged as such and then the offending instruction skipped.  The debug monitor can then check the
       flag to see if its memory access caused a fault or not. */
__MriAdvancePCAndReturn:
    /* Load the current value of the PC into R0 and place the instruction code in R2. */
    ldr     r0, [r1, #24]
    ldrh    r2, [r0]
    /* Add 2 or 4 to the PC value depending on whether the instruction is 16 or 32-bit in length. */
    and     r2, r2, #0xF800
    cmp     r2, #0xE800
    beq     __InstructionIs32Bit
    cmp     r2, #0xF000
    beq     __InstructionIs32Bit
    cmp     r2, #0xF800
    beq     __InstructionIs32Bit
__InstructionIs16Bit:
    add     r0, r0, #2
    b       __StorePCAndReturn
__InstructionIs32Bit:
    add     r0, r0, #4
__StorePCAndReturn:
    str     r0, [r1, #24]
    bx      lr


    .global HardFault_Handler
    .type HardFault_Handler, function
    /* extern "C" void HardFault_Handler(void);
       Override Hard Faults and send to MriExceptionHandler.
    */
HardFault_Handler:
    b       MriExceptionHandler
    
 
    .global MemManage_Handler
    .type MemManage_Handler, function
    /* extern "C" void MemManage_Handler(void);
       Override MPU Memory Faults and send to MriExceptionHandler.
    */
MemManage_Handler:
    b       MriExceptionHandler
    

    .global BusFault_Handler
    .type BusFault_Handler, function
    /* extern "C" void BusFault_Handler(void);
       Override Bus Faults and send to MriExceptionHandler.
    */
BusFault_Handler:
    b       MriExceptionHandler
    

    .global UsageFault_Handler
    .type UsageFault_Handler, function
    /* extern "C" void UsageFault_Handler(void);
       Override Instruction Usage Faults and send to MriExceptionHandler.
    */
UsageFault_Handler:
    b       MriExceptionHandler
    

    .global DebugMon_Handler
    .type DebugMon_Handler, function
    /* extern "C" void DebugMon_Handler(void);
       Override Debug Monintor exceptions and send to MriExceptionHandler.
    */
DebugMon_Handler:
    b       MriExceptionHandler


    .global UART0_IRQHandler
    .type UART0_IRQHandler, function
    /* extern "C" void UART0_IRQHandler(void);
       Override UART0 exceptions and send to MriExceptionHandler.
    */
UART0_IRQHandler:
    b       MriExceptionHandler


    .global __MriSemihostWrite
    .type __MriSemihostWrite, function
    /* extern "C" int __MriSemihostWrite(int file, char *ptr, int len);
       Issues semihost write call to PC via gdb.
    */
__MriSemihostWrite:
    bkpt    0xff
    bx      lr


    .global __MriSemihostRead
    .type __MriSemihostRead, function
    /* extern "C" int __MriSemihostRead(int file, char *ptr, int len);
       Issues semihost read call to PC via gdb.
    */
__MriSemihostRead:
    bkpt    0xff
    bx      lr
    