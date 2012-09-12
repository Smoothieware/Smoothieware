/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Blink LED without using the mbed library. */
#include "LPC17xx.h"

volatile int g_LoopDummy;

int main() 
{
    LPC_GPIO1->FIODIR |= 1 << 18; // P1.18 connected to LED1
    while(1)
    {
        int i;
        
        LPC_GPIO1->FIOPIN ^= 1 << 18; // Toggle P1.18
        for (i = 0 ; i < 5000000 && !g_LoopDummy ; i++)
        {
        }
    }
    return 0;
}
