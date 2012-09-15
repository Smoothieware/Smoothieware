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
/* Basic unit tests for LocalFileSystem functionality. */
#include <mbed.h>

LocalFileSystem local("local");             // Create the local filesystem under the name "local"

int main() 
{
    int  Result = -1;
    long Offset = -1;
    char Buffer[32];
    
    printf("\r\n\r\nGCC4MBED Test Suite\r\n");
    printf("LocalFileSystem Unit Tests\r\n");
    
    printf("Test 1: fopen() for write\r\n");
    FILE *fp = fopen("/local/out.txt", "w");  // Open "out.txt" on the local file system for writing
    if (NULL == fp)
    {
        error("%s(%d) fopen() failed\r\n", __FILE__, __LINE__);
    }

    printf("Test 2: fprintf()\r\n");
    Result = fprintf(fp, "Hello World!");
    if (Result < 0)
    {
        error("%s(%d) fprintf() failed\r\n", __FILE__, __LINE__);
    }

    printf("Test 3: fclose() on written file\r\n");
    Result = fclose(fp);
    if (0 != Result)
    {
        error("%s(%d) fclose() failed\r\n", __FILE__, __LINE__);
    }
    


    printf("Test 4: fopen() for read\r\n");
    fp = fopen("/local/out.txt", "r");
    if (NULL == fp)
    {
        error("%s(%d) fopen() failed\r\n", __FILE__, __LINE__);
    }

    printf("Test 5: fscanf()\r\n");
    Result = fscanf(fp, "%31s", Buffer);
    if (EOF == Result)
    {
        error("%s(%d) fscanf() failed\r\n", __FILE__, __LINE__);
    }
    printf("Contents of /local/out.txt: %s\r\n", Buffer);
    if (0 != strcmp(Buffer, "Hello"))
    {
        error("%s(%d) fscanf read out wrong string\r\n", __FILE__, __LINE__);
    }

    printf("Test 6: Determine size of file through fseek and ftell calls\r\n");
    Result = fseek(fp, 0, SEEK_END);
    if (0 != Result)
    {
        error("%s(%d) fseek(..,0, SEEK_END) failed\r\n", __FILE__, __LINE__);
    }
    Offset = ftell(fp);
    if (12 != Offset)
    {
        error("%s(%d) ftell didn't return the expected value of 12\r\n", __FILE__, __LINE__);
    }
    
    printf("Test 7: fclose() on read file\r\n");
    Result = fclose(fp);
    if (0 != Result)
    {
        error("%s(%d) fclose() failed\r\n", __FILE__, __LINE__);
    }
    


    printf("Test 8: remove()\r\n");
    Result = remove("/local/out.txt");                 // Removes the file "out.txt" from the local file system
    if (0 != Result)
    {
        error("%s(%d) remove() failed\r\n", __FILE__, __LINE__);
    }
    


    if (MRI_ENABLE)
    {
        printf("Skipping dir tests when MRI is enabled as it doesn't support directory tests.\n");
    }
    else
    {
        printf("Test 9: opendir()\r\n");
        DIR *d = opendir("/local");               // Opens the root directory of the local file system
        if (NULL == d)
        {
            error("%s(%d) opendir() failed\r\n", __FILE__, __LINE__);
        }
        struct dirent *p;

        printf("Test 10: readir() for all entries\r\n");
        while((p = readdir(d)) != NULL) 
        {                                         // Print the names of the files in the local file system
          printf("%s\r\n", p->d_name);              // to stdout.
        }

        printf("Test 11: closedir\r\n");
        closedir(d);
    }
    
    printf("\r\nTest completed\r\n");
}
