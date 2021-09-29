#ifndef __STATE_H__
#define __STATE_H__

#include <stdlib.h>

class State
{
    protected:
        int numDimensions;
        double* x;
    public:
        int setNumDimensions(int numDimensionsIn);
        State();
        ~State();
        State(const State& stateIn);
        State& operator=(const State& stateIn);
        double& operator[](const int i) { return x[i]; }

        friend class System;
};


#endif