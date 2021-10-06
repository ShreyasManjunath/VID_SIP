/**
 * \file planner_request_client.cpp
 * */

#include <ros/ros.h>
#include "request_client_node/Request.h"
#include <cstdlib>
#include "tf/tf.h"
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <nav_msgs/Path.h>

std::vector<nav_msgs::Path>* readSTLfile(std::string name);
void setSpaceBoundary(planner_node::inspection& srv);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "request_client_node");
    ROS_INFO("request_client_node Started");
    if(argc != 1)
    {
        ROS_INFO("usage: plan");
        return 1;
    }

    std::string node_name = "/Inspection_Planner";
    std::string mesh_name = "robdekon.stl";

    VID::Request request;

    ros::Rate r1(50.0);
    ros::Rate r2(1.0);

    r2.sleep();

    request.setSpaceBoundary(node_name + "/space_boundary");
    request.setStartingPose(node_name + "/starting_pose");
    request.setPlannerParameters(node_name);
    request.readSTLfile(ros::package::getPath("request_client_node")+"/meshes/" + mesh_name);

    if(!request.start())
    {
        return 1;
    }

    return 0; 
}
