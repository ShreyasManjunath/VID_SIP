/**
 * \file PolygonObject.hpp
 *
 * More elaborate description
 **/
#ifndef __POLYGON_OBJECT_HPP__
#define __POLYGON_OBJECT_HPP__

#include "planner_node/PolygonObject.h"
#include "planner_node/System.h"

VID::Polygon::Polygon()
{
    this->Fixpoint = false;
    this->H = new real_t[3*3];
    this->A = new real_t[3*4];
    this->ubA = new real_t[4];
    this->lbA = new real_t[4];
    this->d = new real_t[3];
}

VID::Polygon::~Polygon()
{
    delete[] H;
    delete[] A;
    delete[] ubA;
    delete[] lbA;
    delete[] d;
}

double VID::Polygon::incidenceAngle = M_PI/3;
double VID::Polygon::minDist = 0;
double VID::Polygon::maxDist = DBL_MAX;
bool VID::Polygon::initialized = false;
std::vector<Vector3f> VID::Polygon::camBoundNormal;

void VID::Polygon::setParam(double incidenceAngleIn, double minDistIn, double maxDistIn)
{
    VID::Polygon::incidenceAngle = incidenceAngleIn;
    VID::Polygon::minDist = minDistIn;
    VID::Polygon::maxDist = maxDistIn;
}

void VID::Polygon::setCamBoundNormals()
{
    VID::Polygon::camBoundNormal.clear();
    Vector3f top(-sin(g_camPitch-g_camAngleVertical/2), 0, -cos(g_camPitch-g_camAngleVertical/2));
    camBoundNormal.push_back(top);
    Vector3f bottom(sin(g_camPitch+g_camAngleVertical/2), 0, cos(g_camPitch+g_camAngleVertical/2));
    camBoundNormal.push_back(bottom);
    Vector3f axis(sin(g_camPitch), 0, cos(g_camPitch));
    AngleAxisf m1 = AngleAxisf(g_camAngleHorizontal/2.0, axis);
    Vector3f y1(0, -1, 0);
    Vector3f left = m1*y1;
    camBoundNormal.push_back(left);
    AngleAxisf m2 = AngleAxisf(-g_camAngleHorizontal/2.0, axis);
    Vector3f y2(0, 1, 0);
    Vector3f right = m2*y2;
    camBoundNormal.push_back(right);
}

void VID::Polygon::setnumOfVertices(int n)
{
    this->numOfVertices = n;
}

int VID::Polygon::getnumOfVertices() const
{
    return this->vertices.size();
}

std::vector<Vector3f> VID::Polygon::getVertices() const
{
    return this->vertices;
}

VID::region* VID::Polygon::IsPolyInCollision(StateVector& stateIn)
{
    double rmin = 0;
    // obstacle check
    for(typename std::list<VID::region*>::iterator iter = System::obstacles.begin(); iter != System::obstacles.end(); iter++)
    {
        if(fabs((*iter)->center[0] - stateIn[0]) < (*iter)->size[0]/2.0 + g_security_distance + rmin &&
            fabs((*iter)->center[1] - stateIn[1]) < (*iter)->size[1]/2.0 + g_security_distance + rmin &&
            fabs((*iter)->center[2] - stateIn[2]) < (*iter)->size[2]/2.0 + g_security_distance)
        {
            return *iter;
        }


    }
    // ray shooting
    for(typename std::list<VID::region*>::iterator iter = System::obstacles.begin(); iter != System::obstacles.end(); iter++)
    {
        if (fabs((*iter)->center[0] - stateIn[0]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        fabs((*iter)->center[1] - stateIn[1]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        fabs((*iter)->center[2] - stateIn[2]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        (*iter)->occupied>0)
        {
            continue;
        }

        // For every vertex of the polygon
        float f_disc = 0.0;
        for(auto& x : vertices)
        {
            f_disc = g_discretization_step / sqrt(SQ(x[0] - stateIn[0]) + SQ(x[1] - stateIn[1]) + SQ(x[2] - stateIn[2]));

            for(float f = 0.0; f < 1.0; f += f_disc)
            {
                if (fabs((*iter)->center[0] - (stateIn[0]*f+(1.0-f)*x[0])) < (*iter)->size[0]/2.0 &&
                    fabs((*iter)->center[1] - (stateIn[1]*f+(1.0-f)*x[1])) < (*iter)->size[1]/2.0 &&
                    fabs((*iter)->center[2] - (stateIn[2]*f+(1.0-f)*x[2])) < (*iter)->size[2]/2.0)
                {
                    return *iter;
                }
            }
        }
        
        
    }
    
    return NULL;
}

#endif