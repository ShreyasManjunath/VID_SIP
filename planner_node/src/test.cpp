/**
 * \file test.cpp
 * Test thread for the PreProcessing
 **/
#include "ros/ros.h"
#include "planner_node/PreProcess.h"
#include <geometry_msgs/Polygon.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_preprocessing_node");
    ros::NodeHandle n;
    PreProcess processObject;

    geometry_msgs::Polygon P;
    geometry_msgs::Point32 p32;

    //  First set of triangles
    p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 8; p32.z = 1.5; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    p32.x = 6.505767; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    p32.x = 2; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    p32.x = 2; p32.y = 2; p32.z = 3; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    p32.x = 24.5; p32.y = 5; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 2; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    p32.x = 24.5; p32.y = 6; p32.z = 4.5; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    //  Second set of triangles

    p32.x = 15.5; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    p32.x = 20; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    p32.x = 20; p32.y = 14; p32.z = 1.5; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    p32.x = 15.5; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    p32.x = 20; p32.y = 14; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 15.5; p32.y = 14; p32.z = 1.5; P.points.push_back(p32);
    processObject.addTriangle(P);
    P.points.clear();

    processObject.setTriOfInterest(1);
    processObject.setThreshArea(9.0);
    processObject.process();

    
    return 0;
}