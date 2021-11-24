#include "planner_node/PreProcess.h"

PreProcess::PreProcess()
{

}

PreProcess::~PreProcess()
{

}

PreProcess::PreProcess(std::vector<geometry_msgs::Polygon>& inspectionArea_in, const float areaThreshold_in)
: areaThreshold{areaThreshold_in}
{
    inspectionArea = inspectionArea_in;
}

void PreProcess::addTriangle(geometry_msgs::Polygon& triangle)
{
    testArea.push_back(triangle);
}

std::tuple<float, Vector3f> PreProcess::getArea(geometry_msgs::Polygon& shape)
{
    std::vector<Vector3f> vertices;
    float area = 0.0;
    Vector3f areaNormal;
    for(auto& p : shape.points)
    {
        vertices.push_back(Vector3f(p.x, p.y, p.z));
    }
    if(vertices.size() == 3)
    {
        Vector3f areaVec = ((vertices[1] - vertices[0]).cross(vertices[2] - vertices[1]))/2;
        area = areaVec.norm();
        areaNormal = areaVec / area;
    }
    return std::make_tuple(area, areaNormal);
}

void PreProcess::initializeTrinagles()
{
    for(auto& tri : testArea)
    {
        auto [area, areaNormal] = getArea(tri);
        ShapeData temp;
        temp.area = area;
        temp.areaNormal = areaNormal;
        temp.shape = tri;
        temp.alreadyFused = false;
    }
}

void PreProcess::process()
{
    initializeTrinagles();
    
}