#include "planner_node/System.h"
#include <cmath>

System::System()
{
    numDimensions = 0;
}

System::~System()
{

}

std::list<VID::region*> System::obstacles(0, NULL);

int System::setNumDimensions(int numDimensionsIn)
{
    if(numDimensions < 0)
        return 0;
    numDimensions = numDimensionsIn;
    rootState.setNumDimensions(numDimensions);
    
    return 1;
}

int System::getStateKey(State& stateIn, double* stateKey)
{
    for(int i = 0; i < numDimensions; i++)
    {
        stateKey[i] = stateIn.x[i] / regionOperating.size[i];
    }

    return 1;
}

bool System::isReachingTarget(State &stateIn)
{
    for (int i = 0; i < numDimensions; i++)
    {
        if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0) 
            return false;
    }
  
    return true;
}

bool System::IsInCollision(double* stateIn)
{
    for(std::list<VID::region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
    {
        if(fabs((*iter)->center[0] - stateIn[0]) < (*iter)->size[0]/2.0 + g_security_distance &&
            fabs((*iter)->center[1] - stateIn[1]) < (*iter)->size[1]/2.0 + g_security_distance &&
            fabs((*iter)->center[2] - stateIn[2]) < (*iter)->size[2]/2.0 + g_security_distance)
        {
            return true;
        }


    }
    return false;
}

int System::sampleState(State& randomStateOut)
{
    randomStateOut.setNumDimensions(numDimensions);

    for(int i = 0; i < numDimensions; i++)
    {
        randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i]
        - regionOperating.size[i]/2.0 + regionOperating.center[i];
    }

    if(IsInCollision(randomStateOut.x))
        return 0;
    
    return 1;
}