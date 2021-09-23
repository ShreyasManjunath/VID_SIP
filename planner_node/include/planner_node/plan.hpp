/**
 * \file plan.hpp
 **/
#ifndef __PLAN_HPP__
#define __PLAN_HPP__

#include <Eigen/Dense>
#include "planner_node/PolygonObject.h"

#define USE_ROTORCRAFT_MODEL

#define __TIMING_INFO__

#define ANGABS(x) (fabs(x)>M_PI?2*M_PI-fabs(x):fabs(x))

#ifdef USE_ROTORCRAFT_MODEL
    typedef Eigen::Vector3f StateVector;
    #define DIMENSIONALITY 4
#endif

typedef VID::Polygon poly_t;

enum koptError_t {
  SUCCESSFUL = 0,               // no error occured, successful execution
  OBSTACLE_INFEASIBILITY,       // the viewpoint is not feasible due to too many obstacles in the sampling region
  CONNECTION_INFEASIBILITY,     // no path could be found due to infeasible conection of viewpoints
  VIEWPOINT_INFEASIBILITY,      // the viewpoint sampling procedure could not find a valid viewpoint
  MISSING_PARAMETER,            // parameter is missing. not loaded?
  NO_START_POSITION,            // no starting position has been defined
  TOO_FEW_INSPECTION_AREAS,     // too few inspection areas have been defined in the service call
  INVALID_OBSTACLE_SHAPE,       // the required obstacle type is not supported
};


#endif

