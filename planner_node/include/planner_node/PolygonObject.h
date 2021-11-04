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
#include "planner_node/Region.h"
#include <list>

typedef Eigen::Vector4f StateVector;
using namespace Eigen;
#define OBSTACLE_DIST_THRESHOLD VID::Polygon::maxDist
#define SQ(x) ((x)*(x))

extern double g_camAngleHorizontal;
extern double g_camAngleVertical;
extern double g_camPitch;
extern double g_max_obs_dim;
extern double g_discretization_step;

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
            Vector3f centroid;
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
            // static std::list<VID::region*> obstacles;
        // Methods
            Polygon();
            ~Polygon();
            static void setParam(double incidenceAngleIn, double minDistIn, double maxDistIn);
            static void setCamBoundNormals();
            int getnumOfVertices() const;
            void setnumOfVertices(int n);
            std::vector<Vector3f> getVertices() const;
            VID::region* IsPolyInCollision(StateVector& stateIn);
            static Vector3f findPolyAreaVector(std::vector<Vector3f> v);
            static Vector3f findUnitNormal(std::vector<Vector3f> v);

    };

}


#endif // __POLYGON_OBJECT_H__
