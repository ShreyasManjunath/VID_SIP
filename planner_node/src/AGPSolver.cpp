#include "planner_node/AGPSolver.h"
#include <fstream>
#include <iostream>

AGPSolver::AGPSolver(poly_t* p, int variables, int constraints)
:m_variables{variables}, m_constraints{constraints}, poly{*p}
{
    this->QPSolver = std::make_unique<QProblem>(QProblem(variables, constraints));
}

void AGPSolver::initPolygon(poly_t* p)
{
    poly_t& poly = *p;
    int numOfVertices = poly.getnumOfVertices();
    if(numOfVertices == 3)
    {
        poly.a = ((poly.vertices[1] - poly.vertices[0]).cross(poly.vertices[3] - poly.vertices[2]))/2;
        poly.aabs = poly.a / poly.a.norm();
        for(int i = 0; i < numOfVertices; i++)
        {
            if(i == numOfVertices-1)
            {
                Vector3f q = poly.vertices[i-(numOfVertices-1)] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                poly.n.push_back(m * poly.a);
            }
            else{
                Vector3f q = poly.vertices[i+1] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                poly.n.push_back(m * poly.a);
            }
        }
    }

}