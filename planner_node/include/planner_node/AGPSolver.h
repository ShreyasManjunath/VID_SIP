#ifndef __AGPSOLVER_H__
#define __AGPSOLVER_H__


#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/package.h>
#include <tuple>
#include <memory>
#include "optec/qpOASES.hpp"
#include "planner_node/plan.hpp"
#include "planner_node/Region.h"

typedef VID::region reg_t;
extern reg_t problemBoundary;
#define X_MIN (problemBoundary.center[0]-problemBoundary.size[0]/2)
#define Y_MIN (problemBoundary.center[1]-problemBoundary.size[1]/2)
#define Z_MIN (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX (problemBoundary.center[0]+problemBoundary.size[0]/2)
#define Y_MAX (problemBoundary.center[1]+problemBoundary.size[1]/2)
#define Z_MAX (problemBoundary.center[2]+problemBoundary.size[2]/2)
double g_const_D = 1.0; // To be moved to Settings file later on.

class AGPSolver
{
    private:
        poly_t& poly;
        int m_variables;
        int m_constraints;
        // Flags to indicate if solutions are found.
        bool solutionFound;
        bool orientationSolutionFound;
        // Index variables
        int currentIndexOfAMatrix;
        int currentIndexOfBoundMatrices;
    public:
        std::unique_ptr<QProblem> QPSolver;
        AGPSolver() = delete;
        AGPSolver(poly_t* p, int variables, int constraints);
        static void initPolygon(poly_t* p);
        ~AGPSolver();
        StateVector dualBarrierSamplerFresh(StateVector* state1, StateVector* state2, StateVector* statePrev);
        void calculateQProblemParams();
        double findOrientationSolution(StateVector& g, StateVector* state1, StateVector* state2);
        std::tuple<StateVector, int> findViewPointSolution(StateVector* state1, StateVector* state2,StateVector* statePrev);
        void setPositionConstraints(bool flag);
        void setDminDmaxConstraint(bool flag);
        void setFOVConstraints(int pw, bool flag);
        bool isVisible(StateVector s);
        bool IsInCollision(StateVector s);
        Vector3f camBoundRotated(Vector3f normal, double roll, double yaw);
        std::vector<Vector2f> locateVerticesOnScreen(std::vector<Vector3f> vertices, Vector3f posInWorld, float yaw);
        float findAreaOfPolyOnScreen(std::vector<Vector2f>& verOnScreen);

        // Static members and methods
        static Matrix3f cameraMtx;

        static void setCameraMtx(std::string node);
};


#endif
