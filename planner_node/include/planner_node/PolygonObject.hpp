/**
 * \file PolygonObject.hpp
 *
 * More elaborate description
 **/
#ifndef __POLYGON_OBJECT_HPP__
#define __POLYGON_OBJECT_HPP__

#include "planner_node/PolygonObject.h"

extern double g_camAngleHorizontal;
extern double g_camAngleVertical;
extern double g_camPitch;

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

#endif