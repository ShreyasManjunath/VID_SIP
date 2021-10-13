#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <stdlib.h>
#include "planner_node/Region.h"
#include "planner_node/plan.hpp"
#include "ros/ros.h"

typedef VID::region reg_t;
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

namespace VID
{
    class Planner
    {
        public:
            // Data members
            double g_camAngleHorizontal;
            double g_camAngleVertical;
            double g_camPitch;
            double minIncidenceAngle;
            double minDist;
            double maxDist;
            int g_convex_pieces;
            double g_angular_discretization_step;
            double g_security_distance;
            double g_max_obs_dim;
            double g_discretization_step;
            double g_cost;

            int maxID;
            StateVector* VP;
            reg_t problemBoundary;
            koptError_t koptError;

            std::vector<poly_t *> polygons;

            Planner();
            ~Planner();
            void plan();
            void preProcess();
        private:
            koptError_t getROSParams();
            void setProblemBoundary();
            std::vector<double> spaceSize;
            std::vector<double> spaceCenter;
    };
}

#endif