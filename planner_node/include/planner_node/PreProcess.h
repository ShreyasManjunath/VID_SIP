#ifndef __PREPROCESS_H__
#define __PREPROCESS_H__

#include <vector>
#include <geometry_msgs/Polygon.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tuple>

using namespace Eigen;

struct ShapeData
{
    geometry_msgs::Polygon shape;
    float area;
    Vector3f areaNormal;
    bool alreadyFused;
};

class PreProcess
{
    private:
        std::vector<geometry_msgs::Polygon> inspectionArea;
        std::vector<geometry_msgs::Polygon> testArea;
        std::vector<ShapeData> initializedTriangles;
        std::vector<geometry_msgs::Polygon> grownArea;
        const float areaThreshold = 6.0;
    public:
        PreProcess();
        ~PreProcess();
        PreProcess(std::vector<geometry_msgs::Polygon>& inspectionArea_in, const float areaThreshold_in);
        void addTriangle(geometry_msgs::Polygon& triangle);
        std::tuple<float, Vector3f> getArea(geometry_msgs::Polygon& shape);
        void initializeTrinagles();
        void process();
        
        
        
};


#endif