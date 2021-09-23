/**
 * \file PolygonObject.hpp
 *
 * More elaborate description
 **/
#ifndef __POLYGON_OBJECT_HPP__
#define __POLYGON_OBJECT_HPP__

#include "planner_node/PolygonObject.h"

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
}

void VID::Polygon::setnumOfVertices(int n)
{
    this->numOfVertices = n;
}

int VID::Polygon::getnumOfVertices() const
{
    return this->numOfVertices;
}

#endif