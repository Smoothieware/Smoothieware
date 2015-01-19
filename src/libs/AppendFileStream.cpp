#include "AppendFileStream.h"
#include <stdio.h>

int AppendFileStream::puts(const char *str)
{
    FILE *fd= fopen(this->fn, "a");
    if(fd == NULL) return 0;

    int n= fwrite(str, 1, strlen(str), fd);
    fclose(fd);
    return n;
}
