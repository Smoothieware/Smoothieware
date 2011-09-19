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
/* Definition of utility routines. */

#ifndef _AGUTIL_H_
#define _AGUTIL_H_


/********
  Macros
 ********/
/* Determines the number of elements in an array */
#define ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]))


/***********************
   Function Prototypes 
 ***********************/
#ifdef __cplusplus
extern "C" {
#endif

void DebugDumpMemory(const void* pvBuffer, size_t BufferSizeInBytes, size_t ElementSize);
void DebugDumpStack(void);

#ifdef __cplusplus
}
#endif


#endif /* _AGUTIL_H_ */
