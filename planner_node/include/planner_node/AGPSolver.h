#ifndef __AGPSOLVER_H__
#define __AGPSOLVER_H__


#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/package.h>
#include <tuple>
#include <memory>
#include "optec/qpOASES.hpp"
#include "planner_node/plan.hpp"
// #include "planner_node/PolygonObject.h"



#define X_MIN (problemBoundary.center[0]-problemBoundary.size[0]/2)
#define Y_MIN (problemBoundary.center[1]-problemBoundary.size[1]/2)
#define Z_MIN (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX (problemBoundary.center[0]+problemBoundary.size[0]/2)
#define Y_MAX (problemBoundary.center[1]+problemBoundary.size[1]/2)
#define Z_MAX (problemBoundary.center[2]+problemBoundary.size[2]/2)


class AGPSolver
{
    private:
        poly_t& poly;
        int m_variables;
        int m_constraints;
    public:
        std::unique_ptr<QProblem> QPSolver;
        AGPSolver() = delete;
        AGPSolver(poly_t* p, int variables, int constraints);
        static void initPolygon(poly_t* p);
};


#endif