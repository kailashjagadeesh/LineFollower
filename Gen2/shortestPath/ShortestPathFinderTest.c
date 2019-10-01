#include<stdio.h>
#include<string.h>
#include "shortest_path.h"

int main()
{
    char SamplePath[]="LBBBBBR";
    FilterPath(SamplePath);
    SimplifyPath(SamplePath, 'L');
    printf("%s", SamplePath);
}
