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
    static const char Filename[] = "/local/foo.bar";
    unsigned char     TestBuffer[256];
    unsigned char     ReadBuffer[256];
    size_t            i;
    
    // Fill in test buffer with every byte value possible.
    for (i = 0 ; i < sizeof(TestBuffer) ; i++)
    {
        TestBuffer[i] = i;
    }
    memset(ReadBuffer, 0, sizeof(ReadBuffer));
    
    // Create a file in the LocalFileSystem with this data.
    FILE* fp = fopen(Filename, "w");
    if (!fp)
    {
        error("Failed to open %s for writing\n", Filename);
    }
    
    int BytesWritten = fwrite(TestBuffer, 1, sizeof(TestBuffer), fp);
    if (BytesWritten != sizeof(TestBuffer))
    {
        error("Failed to write all of the bytes to test file.\n");
    }
    
    fclose(fp);
    fp = NULL;
    
    // Now reopen the file and read in the data and make sure it matches.
    fp = fopen(Filename, "r");
    if (!fp)
    {
        error("Failed to open %s for writing\n", Filename);
    }
    
    int BytesRead = fread(ReadBuffer, 1, sizeof(ReadBuffer), fp);
    if (BytesRead != sizeof(ReadBuffer))
    {
        error("Failed to read all of the bytes from test file.\n");
    }
    if (0 != memcmp(TestBuffer, ReadBuffer, sizeof(TestBuffer)))
    {
        error("File data did match expected value.\n");
    }
    
    fclose(fp);
    fp = NULL;

    // Can delete the file now that we are done with it.
    remove(Filename);

    printf("\r\nTest completed\r\n");
}
