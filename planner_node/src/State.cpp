#include "planner_node/State.h"

State::State()
{
    numDimensions = 0;
    x = NULL;
}

State::~State()
{
    if(x)
        delete[] x;
}

State::State(const State &stateIn)
{
    numDimensions = stateIn.numDimensions;
    if(numDimensions > 0)
    {
        x = new double[numDimensions];
        for(int i = 0; i < numDimensions; i++)
            x[i] = stateIn.x[i];
    }
    else{
        x = NULL;
    }
}

State& State::operator=(const State& stateIn)
{
    if(this == &stateIn)
        return *this;
    if(numDimensions != stateIn.numDimensions)
    {
        if(x)
            delete[] x;
        numDimensions = stateIn.numDimensions;
        if(numDimensions > 0)
            x = new double[numDimensions];
    }
    for(int i = 0; i < numDimensions; i++)
        x[i] = stateIn.x[i];
    
    return *this;
}

int State::setNumDimensions(int numDimensionsIn)
{
    if(x)
        delete[] x;
    if(numDimensions < 0)
        return 0;

    numDimensions = numDimensionsIn;

    if(numDimensions > 0)
        x = new double[numDimensions];
    
    return 1;
}