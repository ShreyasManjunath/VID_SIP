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
        const volatile float areaThreshold = 6.0;
        int triOfIntrst = 0;

    private:
    // Methods
    std::vector<Vector3f> convertMsg2Eigen(geometry_msgs::Polygon t);
    std::vector<Vector3f> setIntersection(std::vector<Vector3f>& A, std::vector<Vector3f>& B);
    std::vector<Vector3f> getUniqueVertices(std::vector<std::vector<Vector3f>>& trianglesToFuse);
    Vector2f project3Dto2D(Vector3f& point, Vector3f& origin, Vector3f& n);
    std::vector<size_t> sortIndexes(const std::vector<float>& v);
    
    public:
        PreProcess();
        ~PreProcess();
        PreProcess(std::vector<geometry_msgs::Polygon>& inspectionArea_in, const float areaThreshold_in);
        void addTriangle(geometry_msgs::Polygon& triangle);
        std::tuple<float, Vector3f> getArea(geometry_msgs::Polygon& shape);
        void initializeTrinagles();
        void process();
        std::tuple<std::vector<std::vector<Vector3f>>, Vector3f> searchForNeighbors(ShapeData& t);
        void setTriOfInterest(int N);
        void setThreshArea(float area) const;
        std::vector<Vector3f> joinVerticesInAntiClock(std::vector<Vector3f>& vertices, Vector3f n);
        
        
};


#endif