#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdlib.h>
#include "planner_node/State.h"
#include "planner_node/Region.h"
#include <list>
extern double g_security_distance;

class State;

class System
{
    protected:
        int numDimensions;
        State rootState;
    public:
        VID::region regionOperating;
        VID::region regionGoal;
        static std::list<VID::region*> obstacles;

        System();
        ~System();
        int setNumDimensions(int numDimensionsIn);
        int getNumDimensions() { return numDimensions; }
        State& getRootState() { return rootState; }
        int getStateKey (State &stateIn, double *stateKey);
        bool isReachingTarget (State &stateIn);
        int sampleState (State &randomStateOut);
        bool IsInCollision(double* stateIn);

};

#endif