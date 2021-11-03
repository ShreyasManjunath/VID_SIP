/**
 * \file plan.cpp
 * Main thread for the planner
 **/

#include "planner_node/plan.hpp"
#include "nav_msgs/Path.h"
#include "planner_node/AGPSolver.h"
#include "planner_node/PolygonObject.h"
#include "planner_node/inspection.h"
#include "ros/ros.h"
#include <geometry_msgs/Polygon.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <fstream>
#include "planner_node/System.h"
#include "planner_node/State.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "tf/tf.h"
#include "planner_node/TSP.h"
#include "planner_node/Planner.h"

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Camera Characteristics
double g_camAngleHorizontal = degreesToRadians(60);
double g_camAngleVertical = degreesToRadians(47);
double g_camPitch = degreesToRadians(30);

int maxID;
StateVector* VP; std::vector<StateVector> VPList;
reg_t problemBoundary;

koptError_t koptError;
std::vector<Vector3f> inputPoly;
double minIncidenceAngle = degreesToRadians(10);
double minDist = 10;
double maxDist = 20;
int g_convex_pieces = 12;
double g_angular_discretization_step = 0.2;
double g_security_distance = 2.0;
double g_max_obs_dim = 0;
double g_discretization_step;
int polygon_seq_count = 0;
int viewpoint_seq_count = 0;
 
double g_cost;
std::vector<double> spaceSize = {200.0, 200.0, 50.0};
std::vector<double> spaceCenter = {0.0, 0.0, 0.0};

std::vector<geometry_msgs::Polygon> inspectionArea;
void visualize(StateVector st);
void drawPolygon(poly_t* tmp);
void drawTrajectory(std::vector<StateVector>& tour);
void readAndInsertPolygon(std::vector<poly_t *>& polygons);
ros::Publisher mesh_pub;
ros::Publisher viewpoint_pub;
ros::Publisher trajectory_pub;

bool plan(planner_node::inspection::Request& req, planner_node::inspection::Response& res)
{
    koptError = SUCCESSFUL;
    std::vector<poly_t *> polygons;
    poly_t::setParam(minIncidenceAngle, minDist, maxDist);
    VID::Polygon::setCamBoundNormals();

    // Starting Point and other required poses
    /*for(std::vector<geometry_msgs::Pose>::iterator itFixPose = req.requiredPoses.begin();
        itFixPose != req.requiredPoses.end() && (itFixPose != req.requiredPoses.end()-1 || req.requiredPoses.size() == 1);
        itFixPose++)
    {
        poly_t *tmp = new poly_t;
        tmp->vertices.push_back(Vector3f((*itFixPose).position.x, (*itFixPose).position.y, (*itFixPose).position.z));
        tf::Pose pose;
        tf::poseMsgToTF(*itFixPose, pose);
        float yaw_angle = tf::getYaw(pose.getRotation());
        tmp->vertices.push_back(Vector3f(yaw_angle, 0.0, 0.0));
        tmp->vertices.push_back(Vector3f(0.0, 0.0, 0.0));
        tmp->Fixpoint = true;
        polygons.push_back(tmp);
    }

    // Read the inspection area (or Mesh data)
    for(std::vector<geometry_msgs::Polygon>::iterator it = req.inspectionArea.begin(); it != req.inspectionArea.end(); it++)
    {
        poly_t *tmp = new poly_t;
        for(auto& vert : (*it).points)
        {
            tmp->vertices.push_back(Vector3f(vert.x, vert.y, vert.z));
        }
        tmp->Fixpoint = false;
        AGPSolver::initPolygon(tmp);
        polygons.push_back(tmp);
        drawPolygon(tmp);
        ros::Duration(0.01).sleep();
    }*/
    readAndInsertPolygon(polygons);

    maxID = polygons.size();
    g_discretization_step = 5.0e-3*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1]));
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
    plannerLog.close();

    for(int i = 0; i < maxID; i++)
    {
        StateVector* s1 = NULL;
        StateVector* s2 = NULL;
        /* sample viewpoint */
        StateVector VPtmp;
        int numOfVertices = polygons[i]->getnumOfVertices();
        int numOfConstraints = numOfVertices + 1 + 4;
        AGPSolver agp(polygons[i], 3, numOfConstraints);
        VPtmp = agp.dualBarrierSamplerFresh(s1, s2, &VP[i]);
        VP[i] = VPtmp; VPList.push_back(VPtmp);
        // visualize(VPtmp);
        /* display sampled viewpoint in rviz */
        visualization_msgs::Marker point;
        point.header.frame_id = "/kopt_frame";
        point.header.stamp = ros::Time::now();
        point.id = i;
        point.ns = "Viewpoints";
        point.type = visualization_msgs::Marker::ARROW;
        point.pose.position.x = VP[i][0];
        point.pose.position.y = VP[i][1];
        point.pose.position.z = VP[i][2];
        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,VP[i][3]);
        point.pose.orientation.x = q.x();
        point.pose.orientation.y = q.y();
        point.pose.orientation.z = q.z();
        point.pose.orientation.w = q.w();
        point.scale.x = 0.4;
        point.scale.y = 0.08;
        point.scale.z = 0.08;
        point.color.r = 1.0f;
        point.color.g = 1.0f;
        point.color.b = 0.2f;
        point.color.a = 0.8;
        point.lifetime = ros::Duration();
        viewpoint_pub.publish(point);
    }

    std::unique_ptr<VID::TSP> tsp_object = std::unique_ptr<VID::TSP>(new VID::TSP{VPList, 0, 1});
    tsp_object->solve();
    auto final_route = tsp_object->getFinalRoute();
    drawTrajectory(final_route);

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
    mesh_pub = n.advertise<nav_msgs::Path>("stl_mesh", 1);
    viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);
    trajectory_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
    ros::Duration(3).sleep();
    geometry_msgs::Polygon P;
    geometry_msgs::Point32 p32;
    // p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    // p32.x = 24.5; p32.y = 5; p32.z = 3; P.points.push_back(p32);
    // p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    // inspectionArea.push_back(P);
    // P.points.clear();
    // p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    // p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    // p32.x = 24.5; p32.y = 8; p32.z = 1.5; P.points.push_back(p32);
    // inspectionArea.push_back(P);
    // P.points.clear();
    p32.x = 2; p32.y = 11; p32.z = 3; P.points.push_back(p32);
    p32.x = 2; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    p32.x = 2; p32.y = 14; p32.z = 1.5; P.points.push_back(p32);
    inspectionArea.push_back(P);
    P.points.clear();
    p32.x = 2; p32.y = 11; p32.z = 3; P.points.push_back(p32);
    p32.x = 2; p32.y = 14; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 2; p32.y = 11; p32.z = 1.5; P.points.push_back(p32);
    inspectionArea.push_back(P);
    P.points.clear();
    p32.x = 15.505768; p32.y = 5; p32.z = 4.5; P.points.push_back(p32);
    p32.x = 15.505768; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    p32.x = 20.005768; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    inspectionArea.push_back(P);
    P.points.clear();
    // p32.x = 20.005768; p32.y = 14; p32.z = 4.5; P.points.push_back(p32);
    // p32.x = 24.500000; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    // p32.x = 20.005768; p32.y = 14; p32.z = 3; P.points.push_back(p32);
    // inspectionArea.push_back(P);
    // P.points.clear();
    // p32.x = 6.505767; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    // p32.x = 2; p32.y = 2; p32.z = 4.5; P.points.push_back(p32);
    // p32.x = 2; p32.y = 2; p32.z = 3; P.points.push_back(p32);
    // inspectionArea.push_back(P);

    // geometry_msgs::Polygon P;
    // geometry_msgs::Point32 p32;
    p32.x = 24.5; p32.y = 8; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 3; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 5; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 8; p32.z = 1.5; P.points.push_back(p32);
    p32.x = 24.5; p32.y = 9; p32.z = 2; P.points.push_back(p32);
    inspectionArea.push_back(P);

    P.points.clear();
    p32.x = 1.86627; p32.y =  4.45662; p32.z =  3.44553; P.points.push_back(p32);
    p32.x = 2.8217; p32.y =  3.11861; p32.z =  2.63698; P.points.push_back(p32);
    p32.x = 4.83986; p32.y =  3.30847; p32.z =  2.68395; P.points.push_back(p32);
    p32.x = 5.40992; p32.y =  5.48551; p32.z = 3.93268; P.points.push_back(p32);
    inspectionArea.push_back(P);

    // plan();
    ros::ServiceServer service = n.advertiseService("inspectionPath", plan);
    ROS_INFO("Service started");
    ros::spin();
    return 0;
}

void visualize(StateVector st)
{
    // Draw Viewpoint
    visualization_msgs::Marker point;
    point.header.frame_id = "/kopt_frame";
    point.header.stamp = ros::Time::now();
    point.header.seq = viewpoint_seq_count++;
    point.ns = "Viewpoints";
    point.type = visualization_msgs::Marker::ARROW;
    point.pose.position.x = st[0];
    point.pose.position.y = st[1];
    point.pose.position.z = st[2];

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, st[3]);
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    point.scale.x = 0.4;
    point.scale.y = 0.08;
    point.scale.z = 0.08;
    point.color.r = 1.0f;
    point.color.g = 1.0f;
    point.color.b = 0.0f;
    point.color.a = 0.7;
    point.lifetime = ros::Duration();
    viewpoint_pub.publish(point);
    ros::Duration(0.01).sleep();
}

void drawPolygon(poly_t* tmp)
{
    // Draw Triangle
    poly_t& poly = *tmp;
    nav_msgs::Path path;
    path.header.frame_id = "/kopt_frame";
    path.header.stamp = ros::Time::now();
    path.header.seq = polygon_seq_count++;
    for(auto& p : poly.vertices)
    {
        geometry_msgs::PoseStamped vert;
        vert.pose.position.x = p[0];
        vert.pose.position.y = p[1];
        vert.pose.position.z = p[2];
        vert.pose.orientation.x =  0.0;
        vert.pose.orientation.y =  0.0;
        vert.pose.orientation.z =  0.0;
        vert.pose.orientation.w =  1.0;
        path.poses.push_back(vert); 
    }
    geometry_msgs::PoseStamped vert;
    vert.pose.position.x = poly.vertices[0][0];
    vert.pose.position.y = poly.vertices[0][1];
    vert.pose.position.z = poly.vertices[0][2];
    vert.pose.orientation.x =  0.0;
    vert.pose.orientation.y =  0.0;
    vert.pose.orientation.z =  0.0;
    vert.pose.orientation.w =  1.0;
    path.poses.push_back(vert);
    mesh_pub.publish(path);
    ros::Duration(0.001).sleep();
}

void drawTrajectory(std::vector<StateVector>& tour)
{
    nav_msgs::Path trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.seq = 1;
    trajectory.header.frame_id = "/kopt_frame";
    int k = 0;
    for(auto& ele : tour)
    {
        geometry_msgs::PoseStamped vp;
        vp.header.frame_id = "/kopt_frame";
        vp.header.seq = k++;
        vp.header.stamp = ros::Time::now();
        vp.pose.position.x = ele[0];
        vp.pose.position.y = ele[1];
        vp.pose.position.z = ele[2];
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, ele[3]);
        vp.pose.orientation.x = q.x();
        vp.pose.orientation.y = q.y();
        vp.pose.orientation.z = q.z();
        vp.pose.orientation.w = q.w();
        trajectory.poses.push_back(vp);
    }
    ros::Duration(0.1).sleep();
    trajectory_pub.publish(trajectory);
}

void readAndInsertPolygon(std::vector<poly_t *>& polygons)
{
    for(std::vector<geometry_msgs::Polygon>::iterator it = inspectionArea.begin(); it != inspectionArea.end(); it++)
    {
        poly_t *tmp = new poly_t;
        for(auto& vert : (*it).points)
        {
            tmp->vertices.push_back(Vector3f(vert.x, vert.y, vert.z));
        }
        tmp->Fixpoint = false;
        AGPSolver::initPolygon(tmp);
        polygons.push_back(tmp);
        drawPolygon(tmp);
        ros::Duration(0.01).sleep();
    }
}