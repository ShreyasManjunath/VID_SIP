#include "planner_node/PreProcess.h"
#include <algorithm>
#include <iterator>
#include <numeric>

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
        initializedTriangles.push_back(temp);
    }
}

void PreProcess::process()
{
    initializeTrinagles();
    auto triangleOfInterest = initializedTriangles[triOfIntrst];

    auto [trianglesToFuse, normal] = searchForNeighbors(triangleOfInterest);
    for(auto& t : trianglesToFuse)
    {
        std::cout << "Triangle:" << std::endl;
        std::cout << "V1: " << t[0].transpose() << std::endl;
        std::cout << "V2: " << t[1].transpose() << std::endl;
        std::cout << "V3: " << t[2].transpose() << std::endl;
    }

    auto allVertices = getUniqueVertices(trianglesToFuse);
    std::cout << "All Vertices :" << std::endl;
    for(auto& v : allVertices)
    {
        std::cout << v.transpose() << std::endl;
    }

    auto joinedVertices = joinVerticesInAntiClock(allVertices, normal);

    std::cout << "Joined Vertices in anti-clock: " << std::endl;
    for(auto& v : joinedVertices)
    {
        std::cout << v.transpose() << std::endl;
    }
    
}

std::tuple<std::vector<std::vector<Vector3f>>, Vector3f> PreProcess::searchForNeighbors(ShapeData& t)
{
    std::vector<std::vector<Vector3f>> trianglesToFuse;
    float totalArea = 0.0;
    totalArea += t.area;
    auto mainTri = convertMsg2Eigen(t.shape); 
    trianglesToFuse.push_back(mainTri);
    for(auto& tri : initializedTriangles)
    {
        std::vector<Vector3f> commonVertices;
        std::vector<Vector3f> tempTri = convertMsg2Eigen(tri.shape);
        if(mainTri == tempTri)
            continue;
        if(t.areaNormal == tri.areaNormal)
        {
            commonVertices = setIntersection(mainTri, tempTri);
            if((int)commonVertices.size() == 2)
            {
                float tempArea = totalArea;
                tempArea += tri.area;
                if(tempArea > this->areaThreshold)
                    continue;
                else
                {
                    totalArea += tri.area;
                    trianglesToFuse.push_back(tempTri);
                }
            }
        }
    }
    std::cout << "Total Area : " << totalArea << std::endl;
    return std::make_tuple(trianglesToFuse, t.areaNormal);
}

std::vector<Vector3f> PreProcess::convertMsg2Eigen(geometry_msgs::Polygon t)
{
    std::vector<Vector3f> retVec;
    for(auto& p : t.points)
    {
        retVec.push_back(Vector3f(p.x, p.y, p.z));
    }
    return retVec;
}

std::vector<Vector3f> PreProcess::setIntersection(std::vector<Vector3f>& A, std::vector<Vector3f>& B)
{
    std::vector<Vector3f> retVec;
    for(auto& p_A : A)
    {
        for(auto& p_B : B)
        {
            if(p_A == p_B)
            {
                retVec.push_back(p_A);
            }
        }
    }
    return retVec;
}

void PreProcess::setTriOfInterest(int N)
{
    triOfIntrst = N -1 ;
}

void PreProcess::setThreshArea(float area) const
{
    float& areaTemp = const_cast<float &>(areaThreshold);
    areaTemp = area;
    std::cout << "AreaThresh: " << areaThreshold << std::endl;
}

std::vector<Vector3f> PreProcess::getUniqueVertices(std::vector<std::vector<Vector3f>>& trianglesToFuse)
{
    std::vector<Vector3f> retVec;
    for(auto& tri : trianglesToFuse)
    {
        for(auto& v : tri)
        {
            if(std::find(retVec.begin(), retVec.end(), v) != retVec.end())
            {
                continue;
            }
            else
            {
                retVec.push_back(v);
            }
        }
    }

    return retVec;
}

Vector2f PreProcess::project3Dto2D(Vector3f& point, Vector3f& origin, Vector3f& n)
{
    Vector3f distVec = point - origin;
    Vector3f projection = distVec - (distVec.dot(n)) * n;
    // setting XY-Plane as the reference 2D plane
    Vector3f xAxis(n[1], -n[0], n[2]);
    Vector3f yAxis = n.cross(xAxis);
    Vector2f pointIn2D(projection.dot(xAxis), projection.dot(yAxis));
    return pointIn2D;
}

std::vector<Vector3f> PreProcess::joinVerticesInAntiClock(std::vector<Vector3f>& vertices, Vector3f n)
{
    std::vector<Vector2f> verticesIn2D;
    std::vector<float> angles;
    Vector3f origin(0.0, 0.0, 0.0);
    //  Making centroid of the polygon as origin.
    for(auto& v : vertices)
    {
        origin += v;
    }
    origin /= vertices.size();

    for(int i = 0; i < vertices.size(); i++)
    {   
        Vector2f temp2DPoint = project3Dto2D(vertices[i], origin, n);
        verticesIn2D.push_back(temp2DPoint);
    }

    auto originIn2D = project3Dto2D(origin, origin, n);
    std::cout << "Origin (centroid) in 2D: " << originIn2D.transpose() << std::endl;
    std::cout << "Vertices in 2D: " << std::endl;
    for(auto& v : verticesIn2D)
    {
        std::cout << v.transpose() << std::endl;
        float tempAngle = std::atan2(v[1] - originIn2D[1], v[0] - originIn2D[0]);
        angles.push_back(tempAngle);
    }

    std::cout << "Angles with centroid reference: " << std::endl;
    for(auto& v : angles)
    {
        std::cout << v << std::endl;
    }

    std::vector<Vector3f> verticesInAntiClock;
    std::cout  << "Index order: [";
    for(auto i : sortIndexes(angles))
    {
        std::cout << i + 1 << " ";
        verticesInAntiClock.push_back(vertices[i]);
    }
    std::cout << "]" << std::endl;

    return verticesInAntiClock;
    
}

std::vector<size_t> PreProcess::sortIndexes(const std::vector<float>& v)
{
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::stable_sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2];});

    return idx;
}