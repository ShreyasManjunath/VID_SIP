#include "planner_node/Planner.h"

VID::Planner::Planner()
{
    koptError = getROSParams();
}

VID::Planner::~Planner()
{

}

void VID::Planner::plan()
{

}

koptError_t VID::Planner::getROSParams()
{
    if(!ros::param::get("~/camera/horizontal", g_camAngleHorizontal))
        koptError = MISSING_PARAMETER;
    if(!ros::param::get("~/camera/vertical", g_camAngleVertical))
        koptError = MISSING_PARAMETER;
    if(!ros::param::get("~/camera/pitch", g_camPitch))
        koptError = MISSING_PARAMETER;
    if(!ros::param::get("~/planner/minDist", minDist))
    {
        ROS_ERROR("minDist missing.");
        koptError = MISSING_PARAMETER;
    }
    if(!ros::param::get("~/planner/maxDist", maxDist))
    {
        ROS_ERROR("maxDist missing.");
        koptError = MISSING_PARAMETER;
    }
    this->setProblemBoundary();

    if(koptError == MISSING_PARAMETER)
    {
        ROS_ERROR("[getROSParams method] Missing Parameter in settings file");
        return MISSING_PARAMETER;
    }
    if(!ros::param::get("~/planner/angular_discretization_step", g_angular_discretization_step))
    {
        g_angular_discretization_step = 0.2;
    }
    if(!ros::param::get("~/planner/security_distance", g_security_distance))
    {
        g_security_distance = 2.0;
    }
    return SUCCESSFUL;;
}

void VID::Planner::setProblemBoundary()
{
    bool flag = true;
    if(!ros::param::get("~/space_boundary/space_size", spaceSize))
    {
        ROS_ERROR("space_size missing.");
        koptError = MISSING_PARAMETER;
        flag = false;
    }
    if(!ros::param::get("~/space_boundary/space_center", spaceCenter))
    {
        ROS_ERROR("space_center missing.");
        koptError = MISSING_PARAMETER;
        flag = false;
    }
    if(flag)
    {
        problemBoundary.setNumDimensions(3);
        assert(spaceCenter.size() == 3 && spaceSize.size());
        for(int i=0; i<3;i++)
        {
            problemBoundary.size[i] = spaceSize[i];
            problemBoundary.center[i] = spaceCenter[i];
        }
        g_discretization_step = 5.0e-3*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1]));
    }
}

void VID::Planner::preProcess()
{
    
}