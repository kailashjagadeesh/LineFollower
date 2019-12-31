#ifndef SOLVER_H
#define SOLVER_H
#include "../JunctionControl/JunctionControl.h"

class Solver {
    static void deleteAndInsert(char *from, const char *insertStr, int pos = 0, int deleteNum = 3);
    public:
    static void solve(char* path, int algo);
};

#endif