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
/* Test which brings default HelloWorld project from mbed online compiler
   to be built under GCC.
*/
#include "mbed.h"

DigitalOut myled(LED1);

int main() 
{
    volatile int IterationCount = 0;
    
    while(1) 
    {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
        IterationCount++;
    }
}
