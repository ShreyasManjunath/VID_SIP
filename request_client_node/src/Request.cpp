#include "request_client_node/Request.h"
#include "tf/tf.h"
#include <geometry_msgs/Polygon.h>
#define Deg2Rad(angleDegrees) (angleDegrees * M_PI / 180.0)

VID::Request::Request()
{
    this->requestClient = nh.serviceClient<planner_node::inspection>("inspectionPath");
}

VID::Request::~Request()
{
    delete mesh; mesh = nullptr;
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
    tf::Quaternion q = tf::createQuaternionFromRPY(Deg2Rad(rot[0]), Deg2Rad(rot[1]), Deg2Rad(rot[2]));
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
    ROS_INFO("mesh size = %i", (int)mesh->size());
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end(); it++)
    {
        geometry_msgs::Polygon p;
        geometry_msgs::Point32 p32;

        p32.x = it->poses[0].pose.position.x;
        p32.y = it->poses[0].pose.position.y;
        p32.z = it->poses[0].pose.position.z;
        p.points.push_back(p32);
        p32.x = it->poses[1].pose.position.x;
        p32.y = it->poses[1].pose.position.y;
        p32.z = it->poses[1].pose.position.z;
        p.points.push_back(p32);
        p32.x = it->poses[2].pose.position.x;
        p32.y = it->poses[2].pose.position.y;
        p32.z = it->poses[2].pose.position.z;
        p.points.push_back(p32);

        srv.request.inspectionArea.push_back(p);
        ros::Rate r(50.0);
        r.sleep();
    }
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
    mesh = new std::vector<nav_msgs::Path>;
    std::fstream f;
    f.open(filename.c_str());
    assert(f.is_open());
    int MaxLine = 0;
    char* line;
    double maxX = -DBL_MAX; double maxY = -DBL_MAX; double maxZ = -DBL_MAX;
    double minX = DBL_MAX; double minY = DBL_MAX; double minZ = DBL_MAX;
    assert(line = (char *) malloc(MaxLine = 80));
    f.getline(line, MaxLine);
    if(0 != strcmp(strtok(line, " "), "solid"))
    {
        ROS_ERROR("Invalid mesh file! Make sure the file is given in ascii-format.");
        ros::shutdown();
    }
    assert(line = (char *) realloc(line, MaxLine));
    f.getline(line, MaxLine);
    int k = 0;
    while(0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown())
    {
        int q = 0;
        nav_msgs::Path p;
        geometry_msgs::PoseStamped v1;
        for(int i = 0; i < 7; i++)
        {
            while(line[q] == ' ')
                q++;
            if(line[q] == 'v')
            {
                const double yawTrafo = 0.0; // used to rotate the mesh before processing
                const double scaleFactor = 1.0; // used to scale the mesh beofore processing
                // Offset the mesh before processing
                const double offsetX = 0.0;
                const double offsetY = 0.0;
                const double offsetZ = 0.0;

                geometry_msgs::PoseStamped vert;
                char* v = strtok(line + q, " ");
                v = strtok(NULL, " ");
                double xtmp = atof(v)/scaleFactor;
                v = strtok(NULL, " ");
                double ytmp = atof(v)/scaleFactor;
                vert.pose.position.x = cos(yawTrafo) * xtmp - sin(yawTrafo) * ytmp;
                vert.pose.position.y = sin(yawTrafo) * xtmp + cos(yawTrafo) * ytmp;
                v = strtok(NULL, " ");
                vert.pose.position.z = atof(v)/scaleFactor;
                vert.pose.position.x -= offsetX;
                vert.pose.position.y -= offsetY;
                vert.pose.position.z -= offsetZ;

                if(maxX < vert.pose.position.x)
                    maxX = vert.pose.position.x;
                if(maxY < vert.pose.position.y)
                    maxY = vert.pose.position.y;
                if(maxZ < vert.pose.position.z)
                    maxZ = vert.pose.position.z;

                if(minX > vert.pose.position.x)
                    minX = vert.pose.position.x;
                if(minY > vert.pose.position.y)
                    minY = vert.pose.position.y;
                if(minZ > vert.pose.position.z)
                    minZ = vert.pose.position.z;
                
                vert.pose.orientation.x = 0.0;
                vert.pose.orientation.y = 0.0;
                vert.pose.orientation.z = 0.0;
                vert.pose.orientation.w = 1.0;
                p.poses.push_back(vert);
                if(p.poses.size() == 1)
                    v1 = vert;
            }
            assert(line = (char *) realloc(line, MaxLine));
            f.getline(line, MaxLine);
        }
        p.poses.push_back(v1);
        p.header.frame_id = "/kopt_frame";
        p.header.stamp = ros::Time::now();
        p.header.seq = k++;
        mesh->push_back(p);
    }
    free(line);
    f.close();
    ROS_INFO("Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
    return mesh;
}