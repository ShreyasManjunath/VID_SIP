#include "request_client_node/Request.h"
#include "tf/tf.h"

VID::Request::Request()
{
    this->requestClient = nh.serviceClient<planner_node::inspection>("inspectionPath");
}

VID::Request::~Request()
{
    
}

void VID::Request::setSpaceBoundary(std::string paramName)
{  
    std::vector<float> mSize;
    std::vector<float> mCenter; 
    ros::param::get(paramName + "/space_size", mSize);
    ros::param::get(paramName + "/space_center", mCenter);
    for(auto ele : mSize)
    {
        srv.request.spaceSize.push_back(ele);
    }
    for(auto ele : mCenter)
    {
        srv.request.spaceCenter.push_back(ele);
    }
}

void VID::Request::setSpaceBoundary(Eigen::Vector3f spaceSizeIn, Eigen::Vector3f spaceCenterIn)
{
    srv.request.spaceSize.push_back(spaceSizeIn[0]);
    srv.request.spaceSize.push_back(spaceSizeIn[1]);
    srv.request.spaceSize.push_back(spaceSizeIn[2]);

    srv.request.spaceCenter.push_back(spaceCenterIn[0]);
    srv.request.spaceCenter.push_back(spaceCenterIn[1]);
    srv.request.spaceCenter.push_back(spaceCenterIn[2]);
}

void VID::Request::setStartingPose(std::string paramName)
{
    std::vector<float> pos;
    std::vector<float> rot;
    bool param_exists = (!ros::param::get(paramName + "/position_xyz", pos)) || (!ros::param::get(paramName + "/rotation_rpy", rot));
    bool data_vectors_empty = pos.empty() || rot.empty();
    bool data_vectors_complete = (pos.size() == 3 || rot.size() == 3);
    if(!param_exists || data_vectors_empty || !data_vectors_complete)
    {
        std::string message = "Starting Pose empty/incomplete: " + paramName + "/position_xyz  or " + paramName + "/rotation_rpy";
        ROS_INFO("%s", message);
        pos = {0.0, 0.0, 5.0}; rot = {0.0 , 0.0, 0.0};
        ROS_INFO("Setting Starting Pose to default pos = [%f, %f, %f], rot = [%f, %f, %f]", 
                 pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
    }
    geometry_msgs::Pose reqPose;
    reqPose.position.x = pos[0];
    reqPose.position.y = pos[1];
    reqPose.position.z = pos[2];
    tf::Quaternion q = tf::createQuaternionFromRPY(rot[0], rot[1], rot[2]);
    reqPose.orientation.x = q.x();
    reqPose.orientation.y = q.y();
    reqPose.orientation.z = q.z();
    reqPose.orientation.w = q.w();
    srv.request.requiredPoses.push_back(reqPose);

}

void VID::Request::setPlannerParameters(std::string paramName)
{
    // Planner Parameters
    float minIncidenceAngle;
    if(!ros::param::get(paramName + "/minIncidenceAngle", minIncidenceAngle)){
        ROS_ERROR("Missing Parameter : minIncidenceAngle");
        minIncidenceAngle = 10.0;
        ROS_INFO("Setting to Default value: minIncidenceAngle = %f", minIncidenceAngle);
    }
    srv.request.incidenceAngle = minIncidenceAngle * M_PI/180.0;

    // Dmin amd Dmax
        // TODO 
}

bool VID::Request::start()
{
    if(requestClient.call(srv))
    {
        ROS_INFO("[Request Client] Request called and suceeded.");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call Request to planner.");
        return false;
    }
}

std::vector<nav_msgs::Path>* VID::Request::readSTLfile(std::string filename)
{
    return NULL;
}