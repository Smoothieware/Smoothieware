/* Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Implementation of routines to help in debugging. */
#include <stdio.h>
#include <string.h>
#include <error.h>
#include <LPC17xx.h>
#include "agutil.h"


/* Dumps the contents of memory to stdout

   Parameters:
    pvBuffer is a pointer to the memory to be dumped
    BufferSizeInBytes is the length of the memory buffer to be dumped
    ElementSize is the size of each element to be dumped.  Valid values are 1, 2, and 4
    
   Returns:
    Nothing
*/
void DebugDumpMemory(const void* pvBuffer, size_t BufferSizeInBytes, size_t ElementSize)
{
    size_t                  i = 0;
    size_t                  Count = BufferSizeInBytes / ElementSize;
    size_t                  ColumnsPerRow = 0;
    size_t                  ByteIndex = 0;
    const unsigned char*    pBuffer = (const unsigned char*)pvBuffer;
    
    char   ASCIIBuffer[9];
    
    /* Check for alignment */
    if ((size_t)pvBuffer & (ElementSize - 1))
    {
        error("RIP: DebugDumpMemory() pvBuffer(%08X) isn't properly aligned to ElementSize(%d)\r\n",
              (size_t)pvBuffer, ElementSize);
        return;
    }
    
    /* Number of elements to dump per line depends on the element size */
    switch(ElementSize)
    {
    case 1:
        ColumnsPerRow = 8;
        break;
    case 2:
        ColumnsPerRow = 16;
        break;
    case 4:
        ColumnsPerRow = 16;
        break;
    default:
        error("RIP: DebugDumpMemory() ElementSize of %u is invalid\r\n", ElementSize);
        return;
    }
    
    
    /* Loop through and dump the elements */
    for (i = 0 ; i < Count ; i++)
    {
        if ((ByteIndex % ColumnsPerRow) == 0)
        {
            printf("%08X: ", (unsigned int)pBuffer);
            memset(ASCIIBuffer, ' ', ARRAY_SIZE(ASCIIBuffer)-1);
            ASCIIBuffer[ARRAY_SIZE(ASCIIBuffer)-1] = '\0';
        }

        switch(ElementSize)
        {
        case 1:
            {
                unsigned char   b;

                b = 0xff & (*(const char*)pBuffer);
                printf("0x%02X ", b);
                ASCIIBuffer[ByteIndex % ColumnsPerRow] = (b >= ' ' && b < 0x80) ? b : '.';
            }
            break;
        case 2:
            printf("0x%04X ", 0xffff & (*(const short*)(const void*)pBuffer));
            break;
        case 4:
            printf("0x%08X ", *(const int*)(const void*)pBuffer);
            break;
        }

        ByteIndex += ElementSize;
        pBuffer   += ElementSize;
        if ((ByteIndex % ColumnsPerRow) == 0)
        {
            printf(" %s\r\n", (1 == ElementSize) ? ASCIIBuffer : "");
        }
    }
    if ((ByteIndex % ColumnsPerRow ) != 0)
    {
        if (1 == ElementSize)
        {
            for ( ; (ByteIndex % ColumnsPerRow) != 0 ; ByteIndex++)
            {
                printf("     ");
            }
            printf(" %s\r\n", ASCIIBuffer);
        }
        else
        {
            printf("\r\n");
        }
    }
}


/* Dumps the contents of the stack to stdout */
void DebugDumpStack(void)
{
    unsigned int TopOfStack = *((unsigned int*)0);
    unsigned int BottomOfStack = __get_MSP();
    unsigned int StackLength = TopOfStack - BottomOfStack;
    
    DebugDumpMemory((void*)BottomOfStack, StackLength, 4);
}
