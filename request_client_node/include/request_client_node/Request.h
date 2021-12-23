#ifndef __REQUEST_H__
#define __REQUEST_H__

#include <stdlib.h>
#include <ros/ros.h>
#include "planner_node/inspection.h"
#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>

namespace VID
{
    class Request
    {
        private:
            planner_node::inspection srv;
            ros::ServiceClient requestClient;
            ros::NodeHandle nh;
            std::vector<nav_msgs::Path>* mesh;
            ros::Publisher stl_pub;
        public:
            Request();
            ~Request();
            std::vector<nav_msgs::Path>* readSTLfile(std::string filename);
            void setSpaceBoundary(std::string paramName);
            void setSpaceBoundary(Eigen::Vector3f spaceSizeIn, Eigen::Vector3f spaceCenterIn);
            void setStartingPose(std::string paramName);
            void setPlannerParameters(std::string paramName);
            bool start();
            
    };
}

#endif