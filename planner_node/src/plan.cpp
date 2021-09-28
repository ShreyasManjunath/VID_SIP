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
#include <fstream>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Camera Characteristics
double g_camAngleHorizontal = degreesToRadians(60);
double g_camAngleVertical = degreesToRadians(47);
double g_camPitch = degreesToRadians(30);

int maxID;
StateVector* VP;
reg_t problemBoundary;

koptError_t koptError;
std::vector<Vector3f> inputPoly;
double minIncidenceAngle = degreesToRadians(10);
double minDist = 3.0;
double maxDist = 8.0;
int g_convex_pieces = 12;
double g_angular_discretization_step = 0.2;

double g_cost;
std::vector<double> spaceSize = {200.0, 200.0, 50.0};
std::vector<double> spaceCenter = {0.0, 0.0, 0.0};


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

    maxID = polygons.size();
    g_cost = DBL_MAX;
    ROS_INFO("Request received");

    problemBoundary.setNumDimensions(3);
    assert(spaceCenter.size() == 3 && spaceSize.size());
    for(int i=0; i<3;i++)
    {
        problemBoundary.size[i] = spaceSize[i];
        problemBoundary.center[i] = spaceCenter[i];
    } 

    if(VP)
    {
        delete[] VP;
    }
    VP = new StateVector[maxID];

    std::string pkgPath = ros::package::getPath("planner_node");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::out);

    for(int i = 0; i < maxID; i++)
    {
        StateVector* s1 = NULL;
        StateVector* s2 = NULL;
        /* sample viewpoint */
        StateVector VPtmp;
        AGPSolver agp(polygons[i], 3, 8);
        VPtmp = agp.dualBarrierSamplerFresh(s1, s2, &VP[i]);
    }


    if(koptError != SUCCESSFUL)
    {
        ROS_ERROR("Error occured! ID: %i", koptError);
        return koptError == SUCCESSFUL;
    }
    delete[] VP;
    VP = NULL;
    ROS_INFO("SUCCESSFUL");
    return koptError == SUCCESSFUL;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;
    inputPoly.push_back(Vector3f(24.5, 8, 3));
    inputPoly.push_back(Vector3f(24.5, 5, 3));
    inputPoly.push_back(Vector3f(24.5 ,5, 1.5));
    plan();
    // ros::ServiceServer service = n.advertiseService("inspectionPath", plan);
    ROS_INFO("Service started");
    ros::spin();
    return 0;
}
