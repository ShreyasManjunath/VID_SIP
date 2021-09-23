/**
 * \file PolygonObject.h
 * Polygon Object class
 **/
#ifndef __POLYGON_OBJECT_H__
#define __POLYGON_OBJECT_H__

#include <eigen3/Eigen/Dense>
#include "optec/qpOASES.hpp"
#include "float.h"
#include <vector>

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace VID
{
    class Polygon
    {
        public:
            bool Fixpoint;
            std::vector<Vector3f> vertices;
            int numOfVertices;
            Vector3f a;
            Vector3f aabs;
            /**
            * \brief Normals of the separating hyperplanes defined by the incidence angle
            * For eg: n1, n2, n3 ...... n#
            **/
            std::vector<Vector3f> n;
            real_t* H;
            real_t* A;
            real_t* lbA;
            real_t* ubA;
            real_t* d;
        // static members
            static double incidenceAngle;
            static double minDist;
            static double maxDist;
            static bool initialized;
            static std::vector<Vector3f> camBoundNormal;
        // Methods
            Polygon();
            ~Polygon();
            static void setParam(double incidenceAngleIn, double minDistIn, double maxDistIn);
            static void setCamBoundNormals();
            int getnumOfVertices() const;
            void setnumOfVertices(int n);
            

    };

}
double VID::Polygon::incidenceAngle = M_PI/3;
double VID::Polygon::minDist = 0;
double VID::Polygon::maxDist = DBL_MAX;
bool VID::Polygon::initialized = false;
std::vector<Vector3f> VID::Polygon::camBoundNormal;

#endif // __POLYGON_OBJECT_H__
