/**
 * \file plan.cpp
 * Main thread for the planner
 **/

#include "planner_node/plan.hpp"
#include "nav_msgs/Path.h"
#include "planner_node/AGPSolver.h"
#include "planner_node/PolygonObject.h"
#include "planner_node/PolygonObject.hpp"
#include "planner_node/inspection.h"
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Int32.h>
#include <stdlib.h>

koptError_t koptError;
std::vector<Vector3f> inputPoly;
double minIncidenceAngle = 10;
double minDist = 5;
double maxDist = 8;

bool plan(/*planner_node::inspection::Request& req, planner_node::inspection::Response& res*/)
{
  koptError = SUCCESSFUL;
  std::vector<poly_t *> polygons;
  poly_t::setParam(minIncidenceAngle, minDist, maxDist);
  poly_t *tmp = new poly_t;
  for (auto &vert : inputPoly) {
    tmp->vertices.push_back(vert);
  }
  AGPSolver::initPolygon(tmp);
  polygons.push_back(tmp);
  return koptError == SUCCESSFUL;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle n;
  inputPoly.push_back(Vector3f(1, 1, 1));
  inputPoly.push_back(Vector3f(2, 2, 2));
  inputPoly.push_back(Vector3f(3, 3, 3));
  plan();
  // ros::ServiceServer service = n.advertiseService("inspectionPath", plan);
  ROS_INFO("Service started");
  ros::spin();
  return 0;
}
