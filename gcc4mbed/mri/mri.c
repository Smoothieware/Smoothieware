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
/* Monitor for Remote Inspection

   Stub implementation to allow Cortex based mbed microcontrollers to be
   debugged from gdb, the GNU debugger.
*/
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>
#include <mbed.h>
#include "mri.h"
#include "mripriv.h"
#include "mriasm.h"
#include "debug_cm3.h"


/*********************/
/* Type Declarations */
/*********************/
/* NOTE: The MriExceptionHandler function definition in mriasm.S is dependent on the layout of this structure.  It
         is also dictated by the version of gdb which supports the ARM processors.  It should only be changed if the
         gdb ARM support code is modified and then the context saving and restoring code will need to be modified to
         use the correct offsets as well. 
*/
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
    /* Reserve room for 8 96-bit floats. */
    unsigned int    Floats[8 * 3];
    unsigned int    FPS;
    unsigned int    CPSR;
} SContext;

typedef struct
{
    char*   pStart;
    char*   pEnd;
    char*   pCurrent;
    int     BufferOverrunDetected;
} SBuffer;

typedef struct
{
    unsigned int    Address;
    int             Is32BitInstruction;
    char            Type;
} SBreakpointWatchpointArguments;

/* NOTE: The largest buffer is required for receiving the 'G' command which receives the contents of the registers from 
   the debugger as two hex digits per byte.  Also need a character for the 'G' command and NULL terminator. */
#define MRI_INPUT_OUTPUT_BUFFER_SIZE (1 + 2 * sizeof(SContext))

typedef struct
{
    SBuffer         Buffer;
    unsigned int    Flags;
    unsigned int    OriginalPC;
    unsigned int    OriginalPSRBitsToMaintain;
    unsigned int    MPUControlValueBeforeDisabling;
    int             SemihostReturnCode;
    int             SemihostErrno;
    int             SignalValue;
    char            InputOutputBuffer[MRI_INPUT_OUTPUT_BUFFER_SIZE];
} SMriState;




/********************************************/
/* RAM based globals used by debug monitor. */
/********************************************/
/* Register context of interrupted program state is stored in this global. */
SContext                g_MriContext;

/* Bitmask of MRI_FLAGS_* bits listed below in Constant Definitions sections. */
volatile unsigned int   g_MriFlags = 0;

/* UID feteched from mbed interface chip before disabling it. */
unsigned char           g_MriMbedUid[36] = "101000000000000000000002F7F00000\0\0\0";

/* UNDONE: Move to stack later when debugger has its own stack. */
/* Tracks state for debugging the current exception. */
static SMriState        g_MriState;

static Serial           g_Serial(USBTX, USBRX);

static char             g_PushedChar;



/************************/
/* Constant Definitions */
/************************/
/* g_MriFlags bit definitions. */
/* NOTE: These flag definitions must match the equivalent .equ's in mriasm.S */
#define     MRI_FLAGS_ACTIVE_DEBUG          1
#define     MRI_FLAGS_FAULT_DURING_DEBUG    2
#define     MRI_FLAGS_FAILED_MBED_DISABLE   4
#define     MRI_FLAGS_MBED_DETECTED         8

/* SBuffer::Flags bit definitions. */
#define     MRI_STATE_FLAGS_SEMIHOST_CTRL_C 1

/* Error strings to be returned to GDB. */
#define     MRI_ERROR_INVALID_ARGUMENT      "E01"   /* Encountered error when parsing command arguments. */
#define     MRI_ERROR_MEMORY_ACCESS_FAILURE "E03"   /* Couldn't access requested memory. */
#define     MRI_ERROR_BUFFER_OVERRUN        "E04"   /* Overflowed internal input/output buffer. */
#define     MRI_ERROR_NO_FREE_BREAKPOINT    "E05"   /* No free FPB breakpoint comparator slots. */

/* The bits that can be set in the return value from a command handler to indicate if the caller should return
   immediately or send the prepared response back to gdb.  It also indicates whether program execution should be
   resumed for commands like continue and single step. */
#define HANDLER_RETURN_RESUME_PROGRAM       1
#define HANDLER_RETURN_RETURN_IMMEDIATELY   2



/***********/
/* Macroes */
/***********/
/* Calculates the number of items in a static array at compile time. */
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

/* Macro to provide index for specified register in the SContext structure. */
#define CONTEXT_MEMBER_INDEX(MEMBER) (offsetof(SContext, MEMBER)/sizeof(unsigned int))

/* Macros to extract upper and lower 4-bit nibbles from 8-bit value. */
#define HI_NIBBLE(X) (((X) >> 4) & 0xF)
#define LO_NIBBLE(X) ((X) & 0xF)



/************************/
/* Function Prototypes. */
/************************/
static void ClearDebuggerActiveFlag(void);
static void GdbCommandHandlingLoop(SMriState* pState);



/***************************************************************/
/* External low-level support routines required by debug stub. */
/***************************************************************/
/* UNDONE: Make these configurable to run different UARTs and at different baud rates. 
           Also move out to a mbed target platform specific file. */
void MriTargetSendChar(int Character)
{
    g_Serial.putc(Character);
}

int MriTargetReceiveChar(void)
{
    return g_Serial.getc();
}

int MriTargetReadable(void)
{
    return g_Serial.readable();
}

int ControlCDetected(void)
{
    static const char ControlC = 0x03;
    char              Char;
    
    while (MriTargetReadable())
    {
        Char = g_Serial.getc();
        if (Char == ControlC)
        {
            return 1;
        }
    }
    
    return 0;
}

static void ConfigureNVICForUartInterrupt(void)
{
    NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);
}

static void EnableUartToInterruptOnReceivedChar(void)
{
    static const uint32_t EnableFIFO_DisableDMA_SetReceiveInterruptThresholdTo0 = 0x01;
    static const uint32_t BaudDivisorLatchBit = (1 << 7);
    static const uint32_t EnableReceiveDataInterrupt = (1 << 0);
    uint32_t              OriginalLCR;
    
    LPC_UART0->FCR = EnableFIFO_DisableDMA_SetReceiveInterruptThresholdTo0;
    OriginalLCR = LPC_UART0->LCR;
    LPC_UART0->LCR &= ~BaudDivisorLatchBit;
    LPC_UART0->IER = EnableReceiveDataInterrupt;
    LPC_UART0->LCR = OriginalLCR;
}

static void InitUart(void)
{
    g_Serial.baud(115200);
    EnableUartToInterruptOnReceivedChar();
    ConfigureNVICForUartInterrupt();
}



/************************************/
/* Hexadecimal Conversion Routines. */
/************************************/
/* Used to convert 4-bit nibble values to hex digit. */
static const char NibbleToHexChar[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

static int HexCharToNibble(unsigned char HexChar)
{
    if (HexChar >= 'a' && HexChar <= 'f')
    {
        return HexChar - 'a' + 10;
    }
    if (HexChar >= 'A' && HexChar <= 'F')
    {
        return HexChar - 'A' + 10;
    }
    if (HexChar >= '0' && HexChar <= '9')
    {
        return HexChar - '0';
    }
    
    /* NOTE: Returns -1 if HexChar isn't a valid hex digit. */
    return -1;
}



/***************************************/
/* Character buffer handling routines. */
/***************************************/
static void SBuffer_Reset(SBuffer* pBuffer)
{
    pBuffer->pCurrent = pBuffer->pStart;
    pBuffer->BufferOverrunDetected = 0;
}

static void SBuffer_Init(SBuffer* pBuffer, char* pBufferStart, size_t BufferSize)
{
    pBuffer->pStart = pBufferStart;
    pBuffer->pEnd = pBufferStart + BufferSize;
    SBuffer_Reset(pBuffer);
}

static size_t SBuffer_BytesLeft(SBuffer* pBuffer)
{
    int BytesLeft = pBuffer->pEnd - pBuffer->pCurrent;
    
    if (BytesLeft < 0)
    {
        pBuffer->BufferOverrunDetected = 1;
        return 0;
    }
    else
    {
        return (size_t)BytesLeft;
    }
}

static void SBuffer_PushBackLastChar(SBuffer* pBuffer)
{
    if (pBuffer->pCurrent > pBuffer->pStart)
    {
        pBuffer->pCurrent--;
    }
}

static int SBuffer_BufferOverrunDetected(SBuffer* pBuffer)
{
    return pBuffer->BufferOverrunDetected;
}

static int SBuffer_WriteChar(SBuffer* pBuffer, char Char)
{
    if (SBuffer_BytesLeft(pBuffer) < 1)
    {
        pBuffer->BufferOverrunDetected = 1;
        return 0;
    }
    else
    {
        *(pBuffer->pCurrent++) = Char;
        return 1;
    }
}

static int SBuffer_ReadChar(SBuffer* pBuffer, char* pChar)
{
    if (SBuffer_BytesLeft(pBuffer) < 1)
    {
        pBuffer->BufferOverrunDetected = 1;
        *pChar = '\0';
        return 0;
    }
    else
    {
        *pChar = *(pBuffer->pCurrent++);
        return 1;
    }
}

static int SBuffer_WriteByteAsHex(SBuffer* pBuffer, unsigned char Byte)
{
    if (SBuffer_BytesLeft(pBuffer) < 2)
    {
        pBuffer->BufferOverrunDetected = 1;
        return 0;
    }
    else
    {
        *(pBuffer->pCurrent++) = NibbleToHexChar[HI_NIBBLE(Byte)];
        *(pBuffer->pCurrent++) = NibbleToHexChar[LO_NIBBLE(Byte)];
        return 1;
    }
}

static int SBuffer_ReadByteAsHex(SBuffer* pBuffer, unsigned char* pByte)
{
    if (SBuffer_BytesLeft(pBuffer) < 2)
    {
        pBuffer->BufferOverrunDetected = 1;
        return 0;
    }
    else
    {
        unsigned char Byte;
    
        Byte = HexCharToNibble(*(pBuffer->pCurrent++)) << 4;
        Byte |= HexCharToNibble(*(pBuffer->pCurrent++));
        *pByte = Byte;
        return 1;
    }
}

static int SBuffer_WriteString(SBuffer* pBuffer, const char* pString)
{
    int Result = 0;
    
    while (*pString)
    {
        Result = SBuffer_WriteChar(pBuffer, *pString++);
        if (!Result)
        {
            return Result;
        }
    }
    
    return 1;
}

static int SBuffer_WriteSizedString(SBuffer* pBuffer, const char* pString, size_t Length)
{
    int Result = 0;
    
    while (Length--)
    {
        Result = SBuffer_WriteChar(pBuffer, *pString++);
        if (!Result)
        {
            return Result;
        }
    }
    
    return 1;
}

static int SBuffer_ReadUIntegerAsHex(SBuffer* pBuffer, unsigned int* pUIntegerResult)
{
    int          HexDigitsParsed = 0;
    unsigned int UInteger = 0;
    char         Char;    

    while (SBuffer_ReadChar(pBuffer, &Char))
    {
        int DigitValue;
        
        DigitValue = HexCharToNibble(Char);
        if (DigitValue < 0)
        {
            /* A character which wasn't a hex digit was encountered so end of hex string has been passed. */
            SBuffer_PushBackLastChar(pBuffer);
            break;
        }

        UInteger = (UInteger << 4) + DigitValue;
        HexDigitsParsed++;
    }

    *pUIntegerResult = UInteger;
    return HexDigitsParsed;
}

static int SBuffer_WriteUIntegerAsHex(SBuffer* pBuffer, unsigned int Value)
{
    unsigned char*  pSrc = ((unsigned char*)&Value) + 3;
    int             NonZero = 0;
    unsigned int    i;
    
    for (i = 0 ; i < sizeof(Value) ; i++)
    {
        unsigned char Byte = *pSrc--;
        
        if (NonZero || Byte)
        {
            SBuffer_WriteByteAsHex(pBuffer, Byte);
            NonZero = 1;
        }
    }
    
    if (!NonZero)
    {
        SBuffer_WriteByteAsHex(pBuffer, 0);
    }
    
    return !SBuffer_BufferOverrunDetected(pBuffer);
}

static int SBuffer_IsNextCharMinus(SBuffer* pBuffer)
{
    if (SBuffer_BytesLeft(pBuffer) == 0)
    {
        /* No bytes left so can't be a minus sign. */
        return 0;
    }
    if (*pBuffer->pCurrent == '-')
    {
        pBuffer->pCurrent++;
        return 1;
    }
    else
    {
        return 0;
    }
}

static int SBuffer_ReadIntegerAsHex(SBuffer* pBuffer, int* pIntegerResult)
{
    int          HexDigitsParsed = 0;
    unsigned int UInteger = 0;
    int          IsNegative;

    IsNegative = SBuffer_IsNextCharMinus(pBuffer);
    HexDigitsParsed = SBuffer_ReadUIntegerAsHex(pBuffer, &UInteger);
    if (UInteger > INT_MAX)
    {
        UInteger = 0;
    }
    if (IsNegative)
    {
        *pIntegerResult = -(int)UInteger;
    }
    else
    {
        *pIntegerResult = (int)UInteger;
    }
    return HexDigitsParsed;
}

static int SBuffer_IsNextCharEqualTo(SBuffer* pBuffer, char ThisChar)
{
    char    NextChar;
    
    if (!SBuffer_ReadChar(pBuffer, &NextChar))
    {
        /* There are no more characters in buffer so it can't be ThisChar. */
        return 0;
    }
    else
    {
        return (ThisChar == NextChar);
    }
}

static int SBuffer_MatchesString(SBuffer* pBuffer, const char* pString, size_t StringLength)
{
    size_t BytesLeft = SBuffer_BytesLeft(pBuffer);
    
    if (BytesLeft < StringLength)
    {
        /* Buffer contents aren't long enough to contain this string so return false. */
        return 0;
    }
    
    if(0 == strncmp(pBuffer->pCurrent, pString, StringLength) &&
       (BytesLeft == StringLength ||
        pBuffer->pCurrent[StringLength] == ':'))
    {
        pBuffer->pCurrent += StringLength;
        return 1;
    }
    else
    {
        return 0;
    }
}



/********************************************************************************************/
/* Routines to send and receive character based checksummed packets to/from the gdb client. */
/********************************************************************************************/
static void SendByteAsHex(unsigned char Byte)
{
    MriTargetSendChar(NibbleToHexChar[HI_NIBBLE(Byte)]);
    MriTargetSendChar(NibbleToHexChar[LO_NIBBLE(Byte)]);
}

static void WaitForStartOfNextPacket(char LastChar)
{
    char Char = LastChar;
    
    /* Wait for the packet start character, '$', and ignore all other characters. */
    while (Char != '$')
    {
        Char = MriTargetReceiveChar();
    }
}

static int GetPacketData(char* pBuffer, size_t BufferSize, unsigned char* pChecksum, char* pLastChar)
{
    char*          pEnd = pBuffer + BufferSize - 1;
    char           Char;
    unsigned char  Checksum;
    char*          pDest;

    /* Read data characters until end of packet character ('#') is encountered, start of next packet character ('$') is
       unexpectedly received or the buffer is unexpectedly filled. */
    pDest = pBuffer;
    Checksum = 0;
    Char = MriTargetReceiveChar();
    while (pDest < pEnd && Char != '$' && Char != '#')
    {
        Checksum = Checksum + (unsigned char)Char;
        *pDest++ = Char;

        Char = MriTargetReceiveChar();
    }
    *pDest = '\0';
    *pChecksum = Checksum;
    *pLastChar = Char;
    
    /* Return success if the expected end of packet character, '#', was received. */
    return (Char == '#');
}

static int ValidatePacketChecksum(unsigned char CalculatedChecksum)
{
    char            ExpectedChecksumString[2];
    unsigned char   ExpectedChecksum;
    SBuffer         Buffer;
    
    ExpectedChecksumString[0] = MriTargetReceiveChar();
    ExpectedChecksumString[1] = MriTargetReceiveChar();
    SBuffer_Init(&Buffer, ExpectedChecksumString, sizeof(ExpectedChecksumString));
    
    SBuffer_ReadByteAsHex(&Buffer, &ExpectedChecksum);
    
    return (ExpectedChecksum == CalculatedChecksum);
}

static void SendACKToGDB(void)
{
    MriTargetSendChar('+');
}

static void SendNAKToGDB(void)
{
    MriTargetSendChar('-');
}

static int GetPacketAndValidateChecksum(char* pBuffer, size_t BufferSize)
{
    char           LastChar;
    unsigned char  CalculatedChecksum;
    int            CompletePacket;
    
    LastChar = g_PushedChar;
    g_PushedChar = '\0';
    do
    {
        WaitForStartOfNextPacket(LastChar);
        CompletePacket = GetPacketData(pBuffer, BufferSize, &CalculatedChecksum, &LastChar);
    } while (!CompletePacket);
    
    return ValidatePacketChecksum(CalculatedChecksum);
}

static int GetMostRecentPacket(char* pBuffer, size_t BufferSize)
{
    int ValidPacket;
    
    do
    {
        ValidPacket = GetPacketAndValidateChecksum(pBuffer, BufferSize);
    } while (MriTargetReadable());
    
    if (ValidPacket)
    {
        SendACKToGDB();
        return 1;
    }
    else
    {
        SendNAKToGDB();
        return 0;
    }
}

static void _GetPacketFromGDB(char* pBuffer, size_t BufferSize)
{
    int     PacketChecksumGood;
    
    do
    {
        PacketChecksumGood = GetMostRecentPacket(pBuffer, BufferSize);
    } while(!PacketChecksumGood);
}

static void _SendPacketToGDB(SBuffer* pBuffer)
{
    char* pEnd = pBuffer->pCurrent;
    char  NextChar;
    
    /* Keeps looping until GDB sends back the '+' packet acknowledge character. */
    do
    {
        unsigned char Checksum = 0;
        char*         pCurrent = pBuffer->pStart;
                
        /* Send packet of format: "$<DataInHex>#<1ByteChecksumInHex> */
        MriTargetSendChar('$');
        while (pCurrent < pEnd)
        {
            char Char = *pCurrent;
            
            MriTargetSendChar(Char);
            Checksum += (unsigned char)Char;
            pCurrent++;
        }

        MriTargetSendChar('#');
        SendByteAsHex(Checksum);
        NextChar = MriTargetReceiveChar();
    } while (NextChar != '+' && NextChar != '$');
    
    if (NextChar == '$')
    {
        /* Started receiving another command packet so return so that it can be parsed even though this packet wasn't
           received fully. */
        g_PushedChar = NextChar;
    }
}



/**************************************************************************************/
/* Routines used to determine cause of exception trap and extract useful information. */
/**************************************************************************************/
static unsigned int GetCurrentlyExecutingExceptionNumber(void)
{
    return (MriGetIPSR() & 0xFF);
}

static unsigned char DetermineCauseOfDebugEvent(void)
{
    struct
    {
        unsigned int    StatusBit;
        unsigned char   SignalToReturn;
    } static const DebugEventToSignalMap[] =
    {
        {SCB_DFSR_EXTERNAL, SIGSTOP},
        {SCB_DFSR_DWTTRAP, SIGTRAP},
        {SCB_DFSR_BKPT, SIGTRAP},
        {SCB_DFSR_HALTED, SIGTRAP}
    };
    unsigned int DebugFaultStatus = SCB->DFSR;
    size_t       i;
    
    for (i = 0 ; i < sizeof(DebugEventToSignalMap)/sizeof(DebugEventToSignalMap[0]) ; i++)
    {
        if (DebugFaultStatus & DebugEventToSignalMap[i].StatusBit)
        {
            SCB->DFSR = DebugEventToSignalMap[i].StatusBit;
            return DebugEventToSignalMap[i].SignalToReturn;
        }
    }
    
    /* NOTE: Default catch all signal is SIGSTOP. */
    return SIGSTOP;
}

static unsigned char DetermineCauseOfException(void)
{
    unsigned int ExceptionNumber = GetCurrentlyExecutingExceptionNumber();
    
    /* UNDONE: Might want to look at Fault Status Registers to better disambiguate cause of faults.  For example you
               can get a UsageFault for divide by 0 errors but today this code just returns SIGILL. */
    switch(ExceptionNumber)
    {
    case 2:
        /* NMI */
        return SIGINT;
    case 3:
        /* HardFault */
        return SIGSEGV;
    case 4:
        /* MemManage */
        return SIGSEGV;
    case 5:
        /* BusFault */
        return SIGBUS;
    case 6:
        /* UsageFault */
        return SIGILL;
    case 12:
        /* Debug Monitor */
        return DetermineCauseOfDebugEvent();
    case 21:
        /* UART0 */
        return SIGINT;
    default:
        /* NOTE: Catch all signal will be SEGSTOP. */
        return SIGSTOP;
    }
}

static unsigned int IsUartInterrupt(void)
{
    return (21 == GetCurrentlyExecutingExceptionNumber());
}



/*********************************************/
/* Routines to manipulate MRI state objects. */
/*********************************************/
static void SMriState_Init(SMriState* pState)
{
    memset(pState, 0, sizeof(*pState));
    pState->SignalValue = DetermineCauseOfException();
    pState->OriginalPC = g_MriContext.PC;
    pState->OriginalPSRBitsToMaintain = g_MriContext.CPSR & PSR_STACK_ALIGN;
    pState->MPUControlValueBeforeDisabling = GetMPUControlValue();
}

static void SMriState_InitBuffer(SMriState* pState)
{
    SBuffer_Init(&pState->Buffer, pState->InputOutputBuffer, sizeof(pState->InputOutputBuffer));
}

static void SMriState_PrepareStringResponse(SMriState* pState, const char* pErrorString)
{
    SMriState_InitBuffer(pState);
    SBuffer_WriteString(&pState->Buffer, pErrorString);
}

static void SMriState_SendPacketToGDB(SMriState* pState)
{
    if (SBuffer_BufferOverrunDetected(&pState->Buffer))
    {
        SMriState_InitBuffer(pState);
        SBuffer_WriteString(&pState->Buffer, MRI_ERROR_BUFFER_OVERRUN);
    }

    _SendPacketToGDB(&pState->Buffer);
}

static void SMriState_GetPacketFromGDB(SMriState* pState)
{
    _GetPacketFromGDB(pState->InputOutputBuffer, sizeof(pState->InputOutputBuffer));
    SBuffer_Init(&pState->Buffer, pState->InputOutputBuffer, strlen(pState->InputOutputBuffer));
}

static void SMriState_PrepareForRestart(SMriState* pState)
{
    SetMPUControlValue(pState->MPUControlValueBeforeDisabling);
    ClearDebuggerActiveFlag();
}

static int SMriState_IsDebugTrap(SMriState* pState)
{
    return (pState->SignalValue == SIGTRAP);
}



/*********************************************/
/* Routines to handle hardcoded breakpoints. */
/*********************************************/
static int IsPCUnchanged(SMriState* pState)
{
    return (g_MriContext.PC == pState->OriginalPC);
}

static int DoesPCPointToHardCodedBreakpoint(void)
{
    static const unsigned short HardCodedBreakpointMachineCode = 0xbe00;
    const unsigned short*       pCurrentInstruction = (const unsigned short*)g_MriContext.PC;
    
    return (HardCodedBreakpointMachineCode == *pCurrentInstruction);
}

static int ShouldSkipHardcodedBreakpoint(SMriState* pState)
{
    return (IsPCUnchanged(pState) && DoesPCPointToHardCodedBreakpoint());
}

static int IsInstruction32Bit(unsigned short FirstWordOfInstruction)
{
    unsigned short MaskedOffUpper5BitsOfWord = FirstWordOfInstruction & 0xF800;
    
    /* 32-bit instructions start with 0b11101, 0b11110, 0b11111 according to page A5-152 of the 
       ARMv7-M Architecture Manual. */
    if (MaskedOffUpper5BitsOfWord == 0xE800 ||
        MaskedOffUpper5BitsOfWord == 0xF000 ||
        MaskedOffUpper5BitsOfWord == 0xF800)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void AdvanceToNextInstruction(void)
{
    unsigned short* pCurrentInstruction;
    unsigned short  FirstWordOfCurrentInstruction;
    
    pCurrentInstruction = (unsigned short*)g_MriContext.PC;
    FirstWordOfCurrentInstruction = *pCurrentInstruction;
    
    if (IsInstruction32Bit(FirstWordOfCurrentInstruction))
    {
        /* 32-bit Instruction. */
        g_MriContext.PC += 4;
    }
    else
    {
        /* 16-bit Instruction. */
        g_MriContext.PC += 2;
    }
}



/***************************************************/
/* Routines to handle mbed semihost functionality. */
/***************************************************/
static unsigned short GetCurrentInstruction(void)
{
    const unsigned short*       pCurrentInstruction = (const unsigned short*)g_MriContext.PC;
    
    return *pCurrentInstruction;
}

static int IsInstructionMbedSemihostBreakpoint(unsigned short Instruction)
{
    static const unsigned short MbedSemihostBreakpointMachineCode = 0xbeab;

    return (MbedSemihostBreakpointMachineCode == Instruction);
}

static int IsInstructionMriSemihostBreakpoint(unsigned short Instruction)
{
    static const unsigned short MriSemihostBreakpointMachineCode = 0xbeff;

    return (MriSemihostBreakpointMachineCode == Instruction);
}

static int DoesPCPointToSemihostBreakpoint(void)
{
    unsigned short Instruction = GetCurrentInstruction();
    
    return (IsInstructionMbedSemihostBreakpoint(Instruction) ||
            IsInstructionMriSemihostBreakpoint(Instruction));
}

extern "C" int semihost_uid(unsigned char* pOutputBuffer);

static int FetchAndSaveMbedUID(void)
{
    return semihost_uid(g_MriMbedUid);
}

static int HandleMbedSemihostUidRequest(void)
{
    struct SUidParameters
    {
        unsigned char*  pBuffer;
        unsigned int    BufferSize;
    };
    const SUidParameters* pParameters;
    unsigned int          CopySize;
    
    pParameters = (const SUidParameters*)g_MriContext.R1;
    CopySize = pParameters->BufferSize;
    if (CopySize > sizeof(g_MriMbedUid))
    {
        CopySize = sizeof(g_MriMbedUid);
    }
    memcpy(pParameters->pBuffer, g_MriMbedUid, CopySize);

    return 0;
}

static int HandleMbedSemihostRequest(void)
{
    unsigned int OpCode;
    int          ReturnCode;
    
    OpCode = g_MriContext.R0;
    switch (OpCode)
    {
    case 257:
        ReturnCode = HandleMbedSemihostUidRequest();
        break;
    default:
        return 0;
    }
    
    AdvanceToNextInstruction();
    
    g_MriContext.R0 = ReturnCode;
    return (ReturnCode != -1);
}

/* Sent when target application wants to write to console or a file on gdb host.  An example is printf().

    Data Format: Fwrite,ff,pp,cc
    
    Where ff is the hex value of the file descriptor of the file to which the data should be written.
          pp is the hex representation of the buffer to be written to the specified file.
          cc is the hex value of the count of bytes in the buffer to be written to the specified file.
*/
static void HandleMriSemihostWriteRequest(SMriState* pState)
{
    static const char  GdbWriteCommand[] = "Fwrite,";
    SBuffer*           pBuffer = &pState->Buffer;
    unsigned int       FileDescriptor = g_MriContext.R0;
    unsigned int       BufferAddress = g_MriContext.R1;
    unsigned int       BufferSize = g_MriContext.R2;

    SMriState_InitBuffer(pState);
    SBuffer_WriteString(pBuffer, GdbWriteCommand);
    SBuffer_WriteUIntegerAsHex(pBuffer,FileDescriptor);
    SBuffer_WriteChar(pBuffer, ',');
    SBuffer_WriteUIntegerAsHex(pBuffer, BufferAddress);
    SBuffer_WriteChar(pBuffer, ',');
    SBuffer_WriteUIntegerAsHex(pBuffer, BufferSize);
    
    SMriState_SendPacketToGDB(pState);
}

/* Sent when target application wants to read from console or a file on gdb host.  An example is scanf().

    Data Format: Fread,ff,pp,cc
    
    Where ff is the hex value of the file descriptor of the file from which the data should be read.
          pp is the hex representation of the buffer to be read into.
          cc is the hex value of the count of bytes in the buffer to be read from the specified file.
*/
static void HandleMriSemihostReadRequest(SMriState* pState)
{
    static const char  GdbReadCommand[] = "Fread,";
    SBuffer*           pBuffer = &pState->Buffer;
    unsigned int       FileDescriptor = g_MriContext.R0;
    unsigned int       BufferAddress = g_MriContext.R1;
    unsigned int       BufferSize = g_MriContext.R2;

    SMriState_InitBuffer(pState);
    SBuffer_WriteString(pBuffer, GdbReadCommand);
    SBuffer_WriteUIntegerAsHex(pBuffer,FileDescriptor);
    SBuffer_WriteChar(pBuffer, ',');
    SBuffer_WriteUIntegerAsHex(pBuffer, BufferAddress);
    SBuffer_WriteChar(pBuffer, ',');
    SBuffer_WriteUIntegerAsHex(pBuffer, BufferSize);
    
    SMriState_SendPacketToGDB(pState);
}

static int WasControlCFlagSet(SMriState* pState)
{
    return (pState->Flags & MRI_STATE_FLAGS_SEMIHOST_CTRL_C);
}

static int WasSemihostCallCancelled(SMriState* pState)
{
    return (pState->SemihostErrno == EINTR);
}

static void MarkSemihostCallAsHandled(SMriState* pState)
{
    AdvanceToNextInstruction();
    g_MriContext.R0 = pState->SemihostReturnCode;
    (errno) = pState->SemihostErrno;
}

static int HandleMriSemihostRequest(SMriState* pState)
{
    unsigned int SemihostOperation;
    
    SemihostOperation = g_MriContext.PC | 1;
    if (SemihostOperation == (unsigned int)__MriSemihostWrite)
    {
        HandleMriSemihostWriteRequest(pState);
    }
    else if (SemihostOperation == (unsigned int)__MriSemihostRead)
    {
        HandleMriSemihostReadRequest(pState);
    }
    else
    {
        return 0;
    }
    
    GdbCommandHandlingLoop(pState);

    if (WasControlCFlagSet(pState))
    {
        if (!WasSemihostCallCancelled(pState))
        {
            MarkSemihostCallAsHandled(pState);
        }
        pState->SignalValue = SIGINT;
        
        return 0;
    }
    else
    {
        MarkSemihostCallAsHandled(pState);

        return 1;
    }
}

static int HandleSemihostRequest(SMriState* pState)
{
    unsigned short Instruction = GetCurrentInstruction();

    if (IsInstructionMbedSemihostBreakpoint(Instruction))
    {
        return HandleMbedSemihostRequest();
    }
    else if (IsInstructionMriSemihostBreakpoint(Instruction))
    {
        return HandleMriSemihostRequest(pState);
    }
    else
    {
        return 0;
    }
}


/***************************************************************************************************/
/* Routines to read/write memory and detect any faults that might occur while attempting to do so. */
/***************************************************************************************************/
static void SetDebuggerActiveFlag(void)
{
    g_MriFlags |= MRI_FLAGS_ACTIVE_DEBUG;
}

static void ClearDebuggerActiveFlag(void)
{
    g_MriFlags &= ~MRI_FLAGS_ACTIVE_DEBUG;
}

static void ClearFaultDetectionFlag(void)
{
    g_MriFlags &= ~MRI_FLAGS_FAULT_DURING_DEBUG;
}

static int WasFaultDetected(void)
{
    return (g_MriFlags & MRI_FLAGS_FAULT_DURING_DEBUG);
}

static int ReadMemoryIntoHexBuffer(SBuffer*    pBuffer,
                                   const void* pvMemory, 
                                   int         ReadByteCount)
{
    const volatile unsigned char* pMemory = (const volatile unsigned char*) pvMemory;
    
    ClearFaultDetectionFlag();

    while (ReadByteCount-- > 0)
    {
        unsigned char Byte;
        
        Byte = *pMemory++;
        if (WasFaultDetected())
        {
            /* Return false when memory access error is encountered. */
            return 0;
        }

        SBuffer_WriteByteAsHex(pBuffer, Byte);
    }

    /* Return true on success.  If pBuffer was overflown, SendPacketToGDB() will detect and return appropriate error. */
    return 1;
}

static int WriteHexBufferToMemory(SBuffer* pBuffer, 
                                  void*    pvMemory, 
                                  int      WriteByteCount)
{
    volatile unsigned char* pMemory = (volatile unsigned char*)pvMemory;

    ClearFaultDetectionFlag();

    while (WriteByteCount-- > 0)
    {
        unsigned char Byte;
    
        if (!SBuffer_ReadByteAsHex(pBuffer, &Byte))
        {
            /* Return false when input hex buffer overflow is detected. */
            return 0;
        }

        *pMemory++ = Byte;
        if (WasFaultDetected())
        {
            /* Return false when memory access error is encountered. */
            return 0;
        }
    }

    /* Return true on successful write. */
    return 1;
}



/***************************************************************/
/* Routines to disable the debug interface on the mbed device. */
/***************************************************************/
static void DisableMbedInterface(void)
{
    static const unsigned int DebugDetachWaitTimeout = 5000;
    
    if (!IsDebuggerAttached())
    {
        /* mbed interface exists on JTAG bus so if no debugger, then no potential for mbed interface. */
        return;
    }
    
    FetchAndSaveMbedUID();
    MriDisableMbed();
    
    if (WaitForDebuggerToDetach(DebugDetachWaitTimeout))
    {
        // UNDONE: printf("Failed to disable mbed debug interface.\n");
        g_MriFlags |= MRI_FLAGS_FAILED_MBED_DISABLE;
    }
    else
    {
        g_MriFlags |= MRI_FLAGS_MBED_DETECTED;
    }
}

static int IsMbedDisabled(void)
{
    return !(g_MriFlags & MRI_FLAGS_FAILED_MBED_DISABLE);
}



/**********************************/
/* GDB Command Handling Routines. */
/**********************************/
static void SendRegisterFor_T_Response(SBuffer* pBuffer, unsigned char RegisterOffset, unsigned int RegisterValue)
{
    SBuffer_WriteByteAsHex(pBuffer, RegisterOffset);
    SBuffer_WriteChar(pBuffer, ':');
    ReadMemoryIntoHexBuffer(pBuffer, &RegisterValue, sizeof(RegisterValue));
    SBuffer_WriteChar(pBuffer, ';');
}

static void PrepareEmptyResponseForUnknownCommand(SMriState* pState)
{
    SMriState_InitBuffer(pState);
}

/* Sent when an exception occurs while program is executing because of previous 'c' (Continue) or 's' (Step) commands.

    Data Format: Tssii:xxxxxxxx;ii:xxxxxxxx;...
    
    Where ss is the hex value of the signal which caused the exception.
          ii is the hex offset of the 32-bit register value following the ':'  The offset is relative to the register
             contents in the g response packet and the SContext structure.
          xxxxxxxx is the 32-bit value of the specified register in hex format.
          The above ii:xxxxxxxx; patterns can be repeated for whichever register values should be sent with T repsonse.
*/
static unsigned int Send_T_StopResponse(SMriState* pState)
{
    SBuffer* pBuffer = &pState->Buffer;
    
    SMriState_InitBuffer(pState);
    SBuffer_WriteChar(pBuffer, 'T');
    SBuffer_WriteByteAsHex(pBuffer, pState->SignalValue);

    SendRegisterFor_T_Response(pBuffer, CONTEXT_MEMBER_INDEX(R12), g_MriContext.R12);
    SendRegisterFor_T_Response(pBuffer, CONTEXT_MEMBER_INDEX(SP), g_MriContext.SP);
    SendRegisterFor_T_Response(pBuffer, CONTEXT_MEMBER_INDEX(LR), g_MriContext.LR);
    SendRegisterFor_T_Response(pBuffer, CONTEXT_MEMBER_INDEX(PC), g_MriContext.PC);

    SMriState_SendPacketToGDB(pState);

    return HANDLER_RETURN_RETURN_IMMEDIATELY;
}

/* Handle the 'g' command which is to send the contents of the registers back to gdb.

    Command Format:     g
    Response Format:    xxxxxxxxyyyyyyyy...
    
    Where xxxxxxxx is the hexadecimal representation of the 32-bit R0 register.
          yyyyyyyy is the hexadecimal representation of the 32-bit R1 register.
          ... and so on through the members of the SContext structure.
*/
static unsigned int HandleRegisterReadCommand(SMriState* pState)
{
    SBuffer* pBuffer = &pState->Buffer;

    SMriState_InitBuffer(pState);
    ReadMemoryIntoHexBuffer(pBuffer, &g_MriContext, sizeof(g_MriContext));
    return 0;
}

/* Handle the 'G' command which is to receive the new contents of the registers from gdb for the program to use when
   it resumes execution.
   
   Command Format:      Gxxxxxxxxyyyyyyyy...
   Response Format:     OK
   
    Where xxxxxxxx is the hexadecimal representation of the 32-bit R0 register.
          yyyyyyyy is the hexadecimal representation of the 32-bit R1 register.
          ... and so on through the members of the SContext structure.
*/
static unsigned int HandleRegisterWriteCommand(SMriState* pState)
{
    SBuffer*        pBuffer = &pState->Buffer;
    int             Result;
    unsigned int    NewPSR;
    
    Result = WriteHexBufferToMemory(pBuffer, &g_MriContext, sizeof(g_MriContext));
    NewPSR = g_MriContext.CPSR & (~PSR_STACK_ALIGN);
    g_MriContext.CPSR = NewPSR | pState->OriginalPSRBitsToMaintain;
    
    SMriState_InitBuffer(pState);
    if (!Result)
    {
        SBuffer_WriteString(pBuffer, MRI_ERROR_BUFFER_OVERRUN);    
    }
    else
    {
        SBuffer_WriteString(pBuffer, "OK");
    }
    return 0;
}

/* Handle the 'm' command which is to read the specified address range from memory.

    Command Format:     mAAAAAAAA,LLLLLLLL
    Response Format:    xx...
    
    Where AAAAAAAA is the hexadecimal representation of the address where the read is to start.
          LLLLLLLL is the hexadecimal representation of the length (in bytes) of the read to be conducted.
          xx is the hexadecimal representation of the first byte read from the specified location.
          ... continue returning the rest of LLLLLLLL-1 bytes in hexadecimal format.
*/
static unsigned int HandleMemoryReadCommand(SMriState* pState)
{
    SBuffer*        pBuffer = &pState->Buffer;
    unsigned int    Address;
    unsigned int    Length;

    if (SBuffer_ReadUIntegerAsHex(pBuffer, &Address) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ',') &&
        SBuffer_ReadUIntegerAsHex(pBuffer, &Length))
    {
        SMriState_InitBuffer(pState);
        if (!ReadMemoryIntoHexBuffer(pBuffer, (unsigned char *)Address, Length))
        {
            /* Received an exception while attempting to read from memory. */
            SMriState_PrepareStringResponse(pState, MRI_ERROR_MEMORY_ACCESS_FAILURE);
        }
    }
    else
    {
        /* Failed parsing the m command arguments. */
        SMriState_PrepareStringResponse(pState, MRI_ERROR_INVALID_ARGUMENT);
    }
    return 0;
}

/* Handle the 'm' command which is to read the specified address range from memory.

    Command Format:     mAAAAAAAA,LLLLLLLL
    Response Format:    xx...
    
    Where AAAAAAAA is the hexadecimal representation of the address where the read is to start.
          LLLLLLLL is the hexadecimal representation of the length (in bytes) of the read to be conducted.
          xx is the hexadecimal representation of the first byte read from the specified location.
          ... continue returning the rest of LLLLLLLL-1 bytes in hexadecimal format.
*/
static unsigned int HandleMemoryWriteCommand(SMriState* pState)
{
    SBuffer*        pBuffer = &pState->Buffer;
    unsigned int    Address;
    unsigned int    Length;

    if (SBuffer_ReadUIntegerAsHex(pBuffer, &Address) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ',') &&
        SBuffer_ReadUIntegerAsHex(pBuffer, &Length) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ':'))
    {
        if (WriteHexBufferToMemory(pBuffer, (unsigned char *)Address, Length))
        {
            /* Successfully wrote to memory so return OK response. */
            SMriState_PrepareStringResponse(pState, "OK");
        }
        else
        {
            /* Received an exception while attempting to write to memory. */
            SMriState_PrepareStringResponse(pState, MRI_ERROR_MEMORY_ACCESS_FAILURE);
        }
    }
    else
    {
        SMriState_PrepareStringResponse(pState, MRI_ERROR_INVALID_ARGUMENT);
    }
    return 0;
}

/* Handle the 'c' command which is sent from gdb to tell the debugger to continue execution of the currently halted
   program.
   
    Command Format:     cAAAAAAAA
    Response Format:    Blank until the next exception, at which time a 'T' stop response packet will be sent.

    Where AAAAAAAA is an optional value to be used for the Program Counter when restarting the program.
*/
static unsigned int HandleContinueCommand(SMriState* pState)
{
    SBuffer* pBuffer = &pState->Buffer;
    unsigned int    Address;

    if (ShouldSkipHardcodedBreakpoint(pState))
    {
        AdvanceToNextInstruction();
    }

    /* Try to read optional parameter, pc unchanged if no parm */
    if (SBuffer_ReadUIntegerAsHex(pBuffer, &Address))
    {
        /* Update PC register with new address. */
        g_MriContext.PC = Address;
    }

    return (HANDLER_RETURN_RESUME_PROGRAM | HANDLER_RETURN_RETURN_IMMEDIATELY);
}

/* Handle the "qSupported" command used by gdb to communicate state to debug monitor and vice versa.

    Reponse Format: qXfer:memory-map:read+;PacketSize==SSSSSSSS
    Where SSSSSSSS is the hexadecimal representation of the maximum packet size support by this stub.
*/
static unsigned int HandleQuerySupportedCommand(SMriState* pState)
{
    /* gdb includes # and 2 chars of checksum that are not placed in our buffer. */
    static const unsigned int PacketSize = sizeof(SContext) + 3;
    static const char         QuerySupportResponse[] = "qXfer:memory-map:read+;PacketSize=";
    SBuffer*                  pBuffer = &pState->Buffer;

    SMriState_InitBuffer(pState);
    SBuffer_WriteString(pBuffer, QuerySupportResponse);
    SBuffer_WriteUIntegerAsHex(pBuffer, PacketSize);
    
    return 0;
}

/* Handle the "qXfer:memory-map" command used by gdb to read the device memory map from the stub.

    Command Format: qXfer:memory-map:read::offset,length
*/
static unsigned int HandleQueryTransferMemoryMapCommand(SMriState* pState)
{
    SBuffer*            pBuffer = &pState->Buffer;
    unsigned int        Offset;
    unsigned int        Length;
    static const char   ReadCommand[] = "read";
    static const char   MemoryMapXML[] = "<?xml version=\"1.0\"?>"
                                         "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                         "<memory-map>"
                                           "<memory type=\"flash\" start=\"0x0\" length=\"0x10000\"> <property name=\"blocksize\">0x1000</property></memory>"    
                                           "<memory type=\"flash\" start=\"0x10000\" length=\"0x70000\"> <property name=\"blocksize\">0x8000</property></memory>"    
                                           "<memory type=\"ram\" start=\"0x10000000\" length=\"0x8000\"> </memory>"    
                                           "<memory type=\"ram\" start=\"0x2007C000\" length=\"0x8000\"> </memory>"    
                                         "</memory-map>";
                                         
    if (SBuffer_IsNextCharEqualTo(pBuffer, ':') &&
        SBuffer_MatchesString(pBuffer, ReadCommand, sizeof(ReadCommand)-1) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ':') &&
        SBuffer_IsNextCharEqualTo(pBuffer, ':') &&
        SBuffer_ReadUIntegerAsHex(pBuffer, &Offset) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ',') &&
        SBuffer_ReadUIntegerAsHex(pBuffer, &Length))
    {
        char         DataPrefixChar = 'm';
        unsigned int OutputBufferSize;
        unsigned int ValidMemoryMapBytes;
        
        if (Offset >= (sizeof(MemoryMapXML) - 1))
        {
            /* Attempt to read past end of XML content so flag with a l only packet. */
            DataPrefixChar = 'l';
            Length = 0;
            ValidMemoryMapBytes = 0;
        }
        else
        {
            ValidMemoryMapBytes = (sizeof(MemoryMapXML) - 1) - Offset;
        }
        
        SMriState_InitBuffer(pState);
        OutputBufferSize = SBuffer_BytesLeft(pBuffer);

        if (Length > OutputBufferSize)
        {
            Length = OutputBufferSize;
        }
        if (Length > ValidMemoryMapBytes)
        {
            DataPrefixChar = 'l';
            Length = ValidMemoryMapBytes;
        }
        
        SBuffer_WriteChar(pBuffer, DataPrefixChar);
        SBuffer_WriteSizedString(pBuffer, &MemoryMapXML[Offset], Length);

        return 0;
    }
    else
    {
        SMriState_PrepareStringResponse(pState, MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }
}

/* Handle the "qXfer" command used by gdb to transfer data to and from the stub for special functionality.

    Command Format: qXfer:object:read:annex:offset,length
    Where supported objects are currently:
        memory-map
*/
static unsigned int HandleQueryTransferCommand(SMriState* pState)
{
    SBuffer*            pBuffer = &pState->Buffer;
    static const char   MemoryMapObject[] = "memory-map";
    
    if (!SBuffer_IsNextCharEqualTo(pBuffer, ':'))
    {
        SMriState_PrepareStringResponse(pState, MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }
    
    if (SBuffer_MatchesString(pBuffer, MemoryMapObject, sizeof(MemoryMapObject)-1))
    {
        return HandleQueryTransferMemoryMapCommand(pState);
    }
    else
    {
        PrepareEmptyResponseForUnknownCommand(pState);
        return 0;
    }
}

/* Handle the 'q' command used by gdb to communicate state to debug monitor and vice versa.

    Command Format: qSSS
    Where SSS is a variable length string indicating which query command is being sent to the stub.
*/
static unsigned int HandleQueryCommand(SMriState* pState)
{
    SBuffer*            pBuffer = &pState->Buffer;
    static const char   qSupportedCommand[] = "Supported";
    static const char   qXferCommand[] = "Xfer";
    
    if (SBuffer_MatchesString(pBuffer, qSupportedCommand, sizeof(qSupportedCommand)-1))
    {
        return HandleQuerySupportedCommand(pState);
    }
    else if (SBuffer_MatchesString(pBuffer, qXferCommand, sizeof(qXferCommand)-1))
    {
        return HandleQueryTransferCommand(pState);
    }
    else
    {
        PrepareEmptyResponseForUnknownCommand(pState);
        return 0;
    }
}

static int Is32BitInstructionKind(char Kind, int* pIs32BitInstruction)
{
    switch (Kind)
    {
    case '2':
        *pIs32BitInstruction = 0;
        return 1;
    case '3':
    case '4':
        *pIs32BitInstruction = 1;
        return 1;
    default:
        *pIs32BitInstruction = 0;
        return 0;
    }
}

static int ParseBreakpointWatchpointCommandArguments(SMriState* pState, SBreakpointWatchpointArguments* pArguments)
{
    SBuffer*    pBuffer = &pState->Buffer;
    char        Kind;
    
    if (SBuffer_ReadChar(pBuffer, &pArguments->Type) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ',') &&
        SBuffer_ReadUIntegerAsHex(pBuffer, &pArguments->Address) &&
        SBuffer_IsNextCharEqualTo(pBuffer, ',') &&
        SBuffer_ReadChar(pBuffer, &Kind) &&
        Is32BitInstructionKind(Kind, &pArguments->Is32BitInstruction))
    {
        return 1;
    }
    else
    {
        SMriState_PrepareStringResponse(pState, MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }
}

/* Handle the '"Z0" command used by gdb to set hardware breakpoints.

    Command Format:     Z0,AAAAAAAA,K
    Response Format:    OK
    Where AAAAAAAA is the hexadecimal representation of the address on which the breakpoint should be set.
          K is either 2: 16-bit Thumb instruction.
                      3: 32-bit Thumb2 instruction.
                      4: 32-bit ARM insruction.
*/
static void HandleHardwareBreakpointSetCommand(SMriState* pState, SBreakpointWatchpointArguments* pArguments)
{
    SBuffer*    pBuffer = &pState->Buffer;
    uint32_t*   pFPBBreakpointComparator;
    
    SMriState_InitBuffer(pState);

    pFPBBreakpointComparator = EnableFPBBreakpoint(pArguments->Address, pArguments->Is32BitInstruction);
    if (!pFPBBreakpointComparator)
    {
        SBuffer_WriteString(pBuffer, MRI_ERROR_NO_FREE_BREAKPOINT);
    }
    else
    {
        SBuffer_WriteString(pBuffer, "OK");
    }
}

/* Handle the '"z0" command used by gdb to remove hardware breakpoints.

    Command Format:     z0,AAAAAAAA,K
    Response Format:    OK
    Where AAAAAAAA is the hexadecimal representation of the address on which the breakpoint should be removed.
          K is either 2: 16-bit Thumb instruction.
                      3: 32-bit Thumb2 instruction.
                      4: 32-bit ARM insruction.
*/
static void HandleHardwareBreakpointRemoveCommand(SMriState* pState, SBreakpointWatchpointArguments* pArguments)
{
    DisableFPBBreakpointComparator(pArguments->Address, pArguments->Is32BitInstruction);
    SMriState_PrepareStringResponse(pState, "OK");
}

/* Handle the Z* commands which can be used to set breakpoints and watchpoints. */
static unsigned int HandleBreakpointWatchpointSetCommand(SMriState* pState)
{
    SBreakpointWatchpointArguments  Arguments;
    
    if (!ParseBreakpointWatchpointCommandArguments(pState, &Arguments))
    {
        return 0;
    }
    
    switch(Arguments.Type)
    {
    case '1':
        HandleHardwareBreakpointSetCommand(pState, &Arguments);
        break;
    default:
        PrepareEmptyResponseForUnknownCommand(pState);
        break;
    }
    
    return 0;
}

/* Handle the z* commands which can be used to remove breakpoints and watchpoints. */
static unsigned int HandleBreakpointWatchpointRemoveCommand(SMriState* pState)
{
    SBreakpointWatchpointArguments  Arguments;
    
    if (!ParseBreakpointWatchpointCommandArguments(pState, &Arguments))
    {
        return 0;
    }
    
    switch(Arguments.Type)
    {
    case '1':
        HandleHardwareBreakpointRemoveCommand(pState, &Arguments);
        break;
    default:
        PrepareEmptyResponseForUnknownCommand(pState);
        break;
    }
    
    return 0;
}

/* Handle the 's' command which is sent from gdb to tell the debugger to single step over the next instruction in the
   currently halted program.
   
    Command Format:     sAAAAAAAA
    Response Format:    Blank until the next exception, at which time a 'T' stop response packet will be sent.

    Where AAAAAAAA is an optional value to be used for the Program Counter when restarting the program.
*/
static unsigned int HandleSingleStepCommand(SMriState* pState)
{
    /* Single step is pretty much like continue except processor is told to only execute 1 instruction. */
    HandleContinueCommand(pState);
    EnableSingleStep();

    return (HANDLER_RETURN_RESUME_PROGRAM | HANDLER_RETURN_RETURN_IMMEDIATELY);
}

static void SetControlCStateFlag(SMriState* pState, int ControlCFlag)
{
    if (ControlCFlag)
    {
        pState->Flags |= MRI_STATE_FLAGS_SEMIHOST_CTRL_C;
    }
    else
    {
        pState->Flags &= ~MRI_STATE_FLAGS_SEMIHOST_CTRL_C;
    }
}

/* Handle the 'F' command which is sent from gdb in response to a previously sent File I/O command from mri.
   
    Command Format:     Frr,ee,C

    Where rr is the hexadecimal representation of the return code from the last requested file I/O command.
          ee is the optional hexadecimal value for the errorno associated with the call if rr indicates error.
          C is the optional 'C' character sent by gdb to indicate that CTRL+C was pressed by user while gdb
            was processing the current file I/O request.
*/
static unsigned int HandleFileIOCommand(SMriState* pState)
{
    static const char ControlCFlag[] = ",C";
    SBuffer* pBuffer = &pState->Buffer;
    int ReturnCode = -1;
    int Errno = 0;
    int ControlC = 0;
    
    SBuffer_ReadIntegerAsHex(pBuffer, &ReturnCode);
    if (SBuffer_IsNextCharEqualTo(pBuffer, ','))
    {
        SBuffer_ReadIntegerAsHex(pBuffer, &Errno);
        ControlC = SBuffer_MatchesString(pBuffer, ControlCFlag, sizeof(ControlCFlag)-1);
    }
    
    pState->SemihostReturnCode = ReturnCode;
    pState->SemihostErrno = Errno;
    SetControlCStateFlag(pState, ControlC);

    return (HANDLER_RETURN_RESUME_PROGRAM | HANDLER_RETURN_RETURN_IMMEDIATELY);
}

static int HandleGDBCommand(SMriState* pState)
{
    SBuffer*        pBuffer = &pState->Buffer;
    unsigned int    HandlerResult = 0;
    char            CommandChar;
    size_t          i;
    static const struct
    {
        unsigned int (*Handler)(SMriState*);
        char         CommandChar;
    } CommandTable[] =
    {
        {Send_T_StopResponse,                       '?'},
        {HandleContinueCommand,                     'c'},
        {HandleFileIOCommand,                       'F'},
        {HandleRegisterReadCommand,                 'g'},
        {HandleRegisterWriteCommand,                'G'},
        {HandleMemoryReadCommand,                   'm'},
        {HandleMemoryWriteCommand,                  'M'},
        {HandleQueryCommand,                        'q'},
        {HandleSingleStepCommand,                   's'},
        {HandleBreakpointWatchpointRemoveCommand,   'z'},
        {HandleBreakpointWatchpointSetCommand,      'Z'}
    };
    
    SMriState_GetPacketFromGDB(pState);
    
    SBuffer_ReadChar(pBuffer, &CommandChar);
    for (i = 0 ; i < ARRAY_SIZE(CommandTable) ; i++)
    {
        if (CommandTable[i].CommandChar == CommandChar)
        {
            HandlerResult = CommandTable[i].Handler(pState);
            if (HandlerResult & HANDLER_RETURN_RETURN_IMMEDIATELY)
            {
                return HandlerResult & HANDLER_RETURN_RESUME_PROGRAM;
            }
            else
            {
                break;
            }
        }
    }
    if (ARRAY_SIZE(CommandTable) == i)
    {
        PrepareEmptyResponseForUnknownCommand(pState);
    }

    SMriState_SendPacketToGDB(pState);
    return (HandlerResult & HANDLER_RETURN_RESUME_PROGRAM);
}

static void GdbCommandHandlingLoop(SMriState* pState)
{
    int StartDebuggeeUpAgain;
    
    do
    {
        StartDebuggeeUpAgain = HandleGDBCommand(pState);
    } while (!StartDebuggeeUpAgain);
}



/********************/
/* Public routines. */
/********************/
extern "C" void MriInit(void)
{
    DisableMbedInterface();
    if (!IsMbedDisabled())
    {
        /* UNDONE: Remove and flag as subsequent error on gdb commands */
        // UNDONE: printf("Exiting MriInit() because of failure to disable mbed device.\n");
        return;
    }
    
    /* UNDONE: Do I need to enable TRACE in the DEMCR to use the DWT or just for it to trace? */
    InitDWT();
    InitFPB();
    
    DisableSingleStep();
    ClearMonitorPending();
    EnableDebugMonitorAtPriority0();
    InitUart();
}


extern "C" void MriDebugException(void)
{
    SetDebuggerActiveFlag();
    if (IsUartInterrupt() && !ControlCDetected())
    {
        /* Just return to active program if gdb sent characters which weren't CTRL+C */
        ClearDebuggerActiveFlag();
        return;
    }
    
    DisableSingleStep();
    ClearMonitorPending();
    SMriState_Init(&g_MriState);
    DisableMPU();
    
    if (SMriState_IsDebugTrap(&g_MriState) && 
        DoesPCPointToSemihostBreakpoint() && 
        HandleSemihostRequest(&g_MriState))
    {
        SMriState_PrepareForRestart(&g_MriState);
        return;
    }
    
    Send_T_StopResponse(&g_MriState);
    GdbCommandHandlingLoop(&g_MriState);

    SMriState_PrepareForRestart(&g_MriState);
}
