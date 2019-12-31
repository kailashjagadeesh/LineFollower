#include "Solver.h"
#include <string.h>
#include <stdio.h>

void Solver::deleteAndInsert(char *from, const char *insertStr, int pos, int deleteNum) {
    sprintf(from, "%s%s", insertStr, from+deleteNum);
}

void Solver::solve(char* path, int algo) {
    bool converting = true;
    int pos = 0;
    while (converting && algo==1) {
        if (strncasecmp(path+pos, "rbl", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (strncasecmp(path+pos, "rbs", 3) == 0) {
            deleteAndInsert(path+pos, "l");pos=0;
        }
        else if (strncasecmp(path+pos, "rbr", 3) == 0) {
            deleteAndInsert(path+pos, "s");pos=0;
        }
        else if (strncasecmp(path+pos, "sbr", 3) == 0) {
            deleteAndInsert(path+pos, "l");pos=0;
        }
        else if (strncasecmp(path+pos, "sbs", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (strncasecmp(path+pos, "lbr", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (pos > strlen(path) - 3) converting = false;
        else pos++;
    }

    while (converting && algo==0) {
        if (strncasecmp(path+pos, "lbr", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (strncasecmp(path+pos, "lbs", 3) == 0) {
            deleteAndInsert(path+pos, "r");pos=0;
        }
        else if (strncasecmp(path+pos, "lbl", 3) == 0) {
            deleteAndInsert(path+pos, "s");pos=0;
        }
        else if (strncasecmp(path+pos, "sbl", 3) == 0) {
            deleteAndInsert(path+pos, "r");pos=0;
        }
        else if (strncasecmp(path+pos, "sbs", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (strncasecmp(path+pos, "rbl", 3) == 0) {
            deleteAndInsert(path+pos, "b");pos=0;
        }
        else if (pos > strlen(path) - 3) converting = false;
        else pos++;
    }
}