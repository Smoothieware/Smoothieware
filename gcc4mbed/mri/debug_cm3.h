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
#ifndef _DEBUG_CM3_H_
#define _DEBUG_CM3_H_

/* Data Watchpoint and Trace Registers */
typedef struct
{
    /* Control register. */
    __IO uint32_t   CTRL;
    /* Cycle Count register. */
    __IO uint32_t   CYCCNT;
    /* CPI Count register. */
    __IO uint32_t   CPICNT;
    /* Exception Overhead Count register. */
    __IO uint32_t   EXCCNT;
    /* Sleep Count register. */
    __IO uint32_t   SLEEPCNT;
    /* Load Store Count register. */
    __IO uint32_t   LSUCNT;
    /* Folded-instruction Count register. */
    __IO uint32_t   FOLDCNT;
    /* Program Counter Sample register. */
    __I  uint32_t   PCSR;
} DWT_Type;

typedef struct
{
    /* Comparator register. */
    __IO uint32_t   COMP;
    /* Comparator Mask register. */
    __IO uint32_t   MASK;
    /* Comparator Function register. */
    __IO uint32_t   FUNCTION;
    /* Reserved 4 bytes to pad struct size out to 16 bytes. */
    __I  uint32_t   Reserved;
} DWT_COMP_Type;

/* Flash Patch and Breakpoint Registers */
typedef struct
{
    /* FlashPatch Control Register. */
    __IO uint32_t   CTRL;
    /* FlashPatch Remap Register. */
    __IO uint32_t   REMAP;
} FPB_Type;


/* Memory mapping of Cortex-M3 Debug Hardware */
#define DWT_BASE        (0xE0001000)
#define DWT_COMP_BASE   (0xE0001020)
#define DWT             ((DWT_Type *)     DWT_BASE)
#define DWT_COMP_ARRAY  ((DWT_COMP_Type*) DWT_COMP_BASE)
#define FPB_BASE        (0xE0002000)
#define FPB_COMP_BASE   (0xE0002008)
#define FPB             ((FPB_Type*) FPB_BASE)
#define FPB_COMP_ARRAY  ((uint32_t*) FPB_COMP_BASE)


/* Debug Halting Control and Status Register Bits */
/*  Enable halt mode debug.  If set to 1 then JTAG debugging is being used. */
#define CoreDebug_DHCSR_C_DEBUGEN   (1 << 0)

/* Debug Exception and Monitor Control Registers Bits */
/*  Monitor Single Step.  Set to 1 to single step instruction when exiting monitor. */
#define CoreDebug_DEMCR_MON_STEP    (1 << 18)
/* Monitor Pending.  Set to 1 to pend a monitor exception. */
#define CoreDebug_DEMCR_MON_PEND    (1 << 17)
/* Monitor Enable.  Set to 1 to enable the debug monitor exception. */
#define CoreDebug_DEMCR_MON_END     (1 << 16)

/* Debug Fault Status Register Bits.  Clear a bit by writing a 1 to it. */
/* Indicates that EDBGRQ was asserted. */
#define SCB_DFSR_EXTERNAL     (1 << 4)
/* Indicates that a vector catch was triggered. */
#define SCB_DFSR_VCATCH       (1 << 3)
/* Indicates that a DWT debug event was triggered. */
#define SCB_DFSR_DWTTRAP      (1 << 2)
/* Indicates a BKPT instruction or FPB match was encountered. */
#define SCB_DFSR_BKPT         (1 << 1)
/* Indicates that a single step has occurred. */
#define SCB_DFSR_HALTED       1

static __INLINE int IsDebuggerAttached(void)
{
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN);
}

static __INLINE int WaitForDebuggerToDetach(unsigned int TimeOut)
{
    while (TimeOut-- > 0 && IsDebuggerAttached())
    {
    }
    
    return IsDebuggerAttached();
}

static __INLINE void EnableDebugMonitorAtPriority0(void)
{
    NVIC_SetPriority(DebugMonitor_IRQn, 0);
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_MON_END;
}

static __INLINE void DisableSingleStep(void)
{
    CoreDebug->DEMCR &=  ~CoreDebug_DEMCR_MON_STEP;
}

static void EnableSingleStep(void)
{
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_MON_STEP;
}

static __INLINE void ClearMonitorPending(void)
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_PEND;
}


/* Data Watchpoint and Trace Comparator Function Bits. */
/*  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read. */
#define DWT_COMP_FUNCTION_MATCHED       (1 << 24)
/*  Data Value Match.  Set to 0 for address compare and 1 for data value compare. */
#define DWT_COMP_FUNCTION_DATAVMATCH    (1 << 8)
/*  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0. */
#define DWT_COMP_FUNCTION_CYCMATCH      (1 << 7)
/*  Enable Data Trace Address offset packets.  0 to disable. */
#define DWT_COMP_FUNCTION_EMITRANGE     (1 << 5)
/*  Selects action to be taken on match. */
#define DWT_COMP_FUNCTION_FUNCTION_MASK             0xF
/*      Disabled */
#define DWT_COMP_FUNCTION_FUNCTION_DISABLED         0x0
/*      Instruction Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_INSTRUCTION      0x4
/*      Data Read Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATA_READ        0x5
/*      Data Write Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE       0x6
/*      Data Read/Write Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATE_READWRITE   0x7

/* DWT - Data Watchpoint Trace Routines */
static __INLINE unsigned int GetDWTComparatorCount(void)
{
    return (DWT->CTRL >> 28);
}

static __INLINE void ClearDWTComparator(DWT_COMP_Type* pComparatorStruct)
{
    pComparatorStruct->COMP = 0;
    pComparatorStruct->MASK = 0;
    pComparatorStruct->FUNCTION &= ~(DWT_COMP_FUNCTION_DATAVMATCH | 
                                     DWT_COMP_FUNCTION_CYCMATCH |
                                     DWT_COMP_FUNCTION_EMITRANGE |
                                     DWT_COMP_FUNCTION_FUNCTION_MASK);
}

static __INLINE void ClearDWTComparators(void)
{
    unsigned int    ComparatorCount;
    unsigned int    i;
    DWT_COMP_Type*  pComparatorStruct = DWT_COMP_ARRAY;
    
    ComparatorCount = GetDWTComparatorCount();
    
    for (i = 0 ; i < ComparatorCount ; i++)
    {
        ClearDWTComparator(pComparatorStruct);
        pComparatorStruct++;
    }
}

static __INLINE void InitDWT(void)
{
    ClearDWTComparators();
}


/* FlashPatch Control Register Bits. */
/*  Most significant bits of number of instruction address comparators.  Read-only */
#define FP_CTRL_NUM_CODE_MSB_SHIFT  12
#define FP_CTRL_NUM_CODE_MSB_MASK   (0x7 << FP_CTRL_NUM_CODE_MSB_SHIFT)
/*  Least significant bits of number of instruction address comparators.  Read-only */
#define FP_CTRL_NUM_CODE_LSB_SHIFT  4
#define FP_CTRL_NUM_CODE_LSB_MASK   (0xF << FP_CTRL_NUM_CODE_LSB_SHIFT)
/*  Number of instruction literal address comparators.  Read only */
#define FP_CTRL_NUM_LIT_SHIFT       8
#define FP_CTRL_NUM_LIT_MASK        (0xF << FP_CTRL_NUM_LIT_SHIFT)
/*  This Key field must be set to 1 when writing or the write will be ignored. */
#define FP_CTRL_KEY                 (1 << 1)
/*  Enable bit for the FPB.  Set to 1 to enable FPB. */
#define FP_CTRL_ENABLE              1

/* FlashPatch Comparator Register Bits. */
/*  Defines the behaviour for code address comparators. */
#define FP_COMP_REPLACE_SHIFT       30
#define FP_COMP_REPLACE_MASK        (0x3 << FP_COMP_REPLACE_SHIFT)
/*      Remap to specified address in SRAM. */
#define FP_COMP_REPLACE_REMAP       (0x0 << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on lower halfword. */
#define FP_COMP_REPLACE_BREAK_LOWER (0x1 << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on upper halfword. */
#define FP_COMP_REPLACE_BREAK_UPPER (0x2 << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on word. */
#define FP_COMP_REPLACE_BREAK       (0x3 << FP_COMP_REPLACE_SHIFT)
/*  Specified bits 28:2 of the address to be use for match on this comparator. */
#define FP_COMP_COMP_SHIFT          2
#define FP_COMP_COMP_MASK           (0x07FFFFFF << FP_COMP_COMP_SHIFT)
/*  Enables this comparator.  Set to 1 to enable. */
#define FP_COMP_ENABLE              1

/* FPB - Flash Patch Breakpoint Routines. */
static __INLINE unsigned int GetFPBCodeComparatorCount(void)
{
    uint32_t    ControlValue = FPB->CTRL;
    return (((ControlValue & FP_CTRL_NUM_CODE_MSB_MASK) >> 8) |
            ((ControlValue & FP_CTRL_NUM_CODE_LSB_MASK) >> 4));
}

static __INLINE unsigned int GetFPBLiteralComparatorCount(void)
{
    uint32_t    ControlValue = FPB->CTRL;
    return ((ControlValue & FP_CTRL_NUM_LIT_MASK) >> FP_CTRL_NUM_LIT_SHIFT);
}

static __INLINE void ClearFPBComparator(uint32_t* pComparator)
{
    *pComparator = 0;
}

static __INLINE int32_t IsAddressInUpperHalfGig(uint32_t Address)
{
    return (Address & 0xE0000000);
}

static __INLINE int32_t IsAddressOdd(uint32_t Address)
{
    return (Address & 0x1);
}

static __INLINE int32_t IsBreakpointAddressInvalid(uint32_t BreakpointAddress)
{
     return (IsAddressInUpperHalfGig(BreakpointAddress) || IsAddressOdd(BreakpointAddress));
}

static __INLINE uint32_t IsAddressInUpperHalfword(uint32_t Address)
{
    return (Address & 0x2);
}

static __INLINE uint32_t CalculateFPBComparatorReplaceValue(uint32_t BreakpointAddress, int32_t Is32BitInstruction)
{
    if (Is32BitInstruction)
    {
        return FP_COMP_REPLACE_BREAK;
    }
    else if (IsAddressInUpperHalfword(BreakpointAddress))
    {
        return FP_COMP_REPLACE_BREAK_UPPER;
    }
    else
    {
        return FP_COMP_REPLACE_BREAK_LOWER;
    }
}

static __INLINE uint32_t CalculateFPBComparatorValue(uint32_t BreakpointAddress, int32_t Is32BitInstruction)
{
    uint32_t    ComparatorValue;
    
    if (IsBreakpointAddressInvalid(BreakpointAddress))
    {
        /* Can only set a breakpoint on addresses where the upper 3-bits are all 0 (upper 0.5GB is off limits) and 
           the address is half-word aligned */
        return ~0UL;
    }
    
    ComparatorValue = (BreakpointAddress & FP_COMP_COMP_MASK);
    ComparatorValue |= FP_COMP_ENABLE;
    ComparatorValue |= CalculateFPBComparatorReplaceValue(BreakpointAddress, Is32BitInstruction);
    
    return ComparatorValue;
}

static __INLINE uint32_t MaskOffFPBComparatorReservedBits(uint32_t ComparatorValue)
{
    return (ComparatorValue & (FP_COMP_REPLACE_MASK | FP_COMP_COMP_MASK | FP_COMP_ENABLE));
}

static __INLINE uint32_t IsFPBComparatorEnabled(uint32_t Comparator)
{
    return (Comparator & FP_COMP_ENABLE);
}

static __INLINE uint32_t* FindFPBBreakpointComparator(uint32_t BreakpointAddress, int32_t Is32BitInstruction)
{
    uint32_t*    pCurrentComparator = FPB_COMP_ARRAY;
    uint32_t     ComparatorValueForThisBreakpoint;
    unsigned int CodeComparatorCount;
    unsigned int i;
    
    ComparatorValueForThisBreakpoint = CalculateFPBComparatorValue(BreakpointAddress, Is32BitInstruction);
    CodeComparatorCount = GetFPBCodeComparatorCount();
    
    for (i = 0 ; i < CodeComparatorCount ; i++)
    {
        uint32_t MaskOffReservedBits;
        
        MaskOffReservedBits = MaskOffFPBComparatorReservedBits(*pCurrentComparator);
        if (ComparatorValueForThisBreakpoint == MaskOffReservedBits)
        {
            return pCurrentComparator;
        }
        pCurrentComparator++;
    }
    
    /* Return NULL if no FPB comparator is already enabled for this breakpoint. */
    return NULL;
}

static __INLINE uint32_t* FindFreeFPBBreakpointComparator(void)
{
    uint32_t*    pCurrentComparator = FPB_COMP_ARRAY;
    unsigned int CodeComparatorCount;
    unsigned int i;
    
    CodeComparatorCount = GetFPBCodeComparatorCount();

    for (i = 0 ; i < CodeComparatorCount ; i++)
    {
        if (!IsFPBComparatorEnabled(*pCurrentComparator))
        {
            return pCurrentComparator;
        }
        pCurrentComparator++;
    }
    
    /* Return NULL if no FPB breakpoint comparators are free. */
    return NULL;
}

static __INLINE uint32_t* EnableFPBBreakpoint(uint32_t BreakpointAddress, int32_t Is32BitInstruction)
{
    uint32_t* pExistingFPBBreakpoint;
    uint32_t* pFreeFPBBreakpointComparator;
    
    pExistingFPBBreakpoint = FindFPBBreakpointComparator(BreakpointAddress, Is32BitInstruction);
    if (pExistingFPBBreakpoint)
    {
        /* This breakpoint is already set to just return pointer to existing comparator. */
        return pExistingFPBBreakpoint;
    }
    
    pFreeFPBBreakpointComparator = FindFreeFPBBreakpointComparator();
    if (!pFreeFPBBreakpointComparator)
    {
        /* All FPB breakpoint comparator slots are used so return NULL as error indicator. */
        return NULL;
    }
    
    
    *pFreeFPBBreakpointComparator = CalculateFPBComparatorValue(BreakpointAddress, Is32BitInstruction);
    return pFreeFPBBreakpointComparator;
}

static __INLINE uint32_t* DisableFPBBreakpointComparator(uint32_t BreakpointAddress, int32_t Is32BitInstruction)
{
    uint32_t* pExistingFPBBreakpoint;
    
    pExistingFPBBreakpoint = FindFPBBreakpointComparator(BreakpointAddress, Is32BitInstruction);
    if (pExistingFPBBreakpoint)
    {
        ClearFPBComparator(pExistingFPBBreakpoint);
    }

    return pExistingFPBBreakpoint;
}

static __INLINE void ClearFPBComparators(void)
{
    unsigned int CodeComparatorCount;
    unsigned int LiteralComparatorCount;
    unsigned int TotalComparatorCount;
    unsigned int i;
    uint32_t*    pCurrentComparator = FPB_COMP_ARRAY;
    
    CodeComparatorCount = GetFPBCodeComparatorCount();
    LiteralComparatorCount = GetFPBLiteralComparatorCount();
    TotalComparatorCount = CodeComparatorCount + LiteralComparatorCount;
    
    for (i = 0 ; i < TotalComparatorCount ; i++)
    {
        ClearFPBComparator(pCurrentComparator);
        pCurrentComparator++;
    }
}

static __INLINE void EnableFPB(void)
{
    FPB->CTRL |= (FP_CTRL_KEY | FP_CTRL_ENABLE);
}

static __INLINE void InitFPB(void)
{
    ClearFPBComparators();
    EnableFPB();
}


/* Memory Protection Unit Type Register Bits. */
/* Number of instruction regions supported by MPU.  0 for Cortex-M3 */
#define MPU_TYPE_IREGION_SHIFT      16
#define MPU_TYPE_IREGION_MASK       (0xFF << MPU_TYPE_IREGION_SHIFT)
/* Number of data regions supported by MPU. */
#define MPU_TYPE_DREGION_SHIFT      8
#define MPU_TYPE_DREGION_MASK       (0xFF << MPU_TYPE_DREGION_SHIFT)
/* Are instruction and data regions configured separately?  1 for yes and 0 otherwise. */
#define MPU_TYPE_SEPARATE           0x1

/* Memory Protection Unit Control Register Bits. */
/* Default memory map as background region for privileged access. 1 enables. */
#define MPU_CTRL_PRIVDEFENA         (1 << 2)
/* Hard fault and NMI exceptions to use MPU. 0 disables MPU for these handlers. */
#define MPU_CTRL_HFNMIENA           (1 << 1)
/* MPU Enable.  1 enables and disabled otherwise. */
#define MPU_CTRL_ENABLE             1

/* MPU - Memory Protection Unit Routines. */
static __INLINE unsigned int GetMPUControlValue(void)
{
    return (MPU->CTRL);
}

static __INLINE void SetMPUControlValue(unsigned int NewControlValue)
{
    MPU->CTRL = NewControlValue;
}

static __INLINE void DisableMPU(void)
{
    MPU->CTRL &= ~MPU_CTRL_ENABLE;
}


/* Program Status Register Bits. */
/*  Was the stack 8-byte aligned during auto stacking. */
#define PSR_STACK_ALIGN     (1 << 9)


#endif /* _DEBUG_CM3_H_ */
