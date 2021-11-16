#include "planner_node/AGPSolver.h"
#include <fstream>
#include <iostream>
#include "planner_node/System.h"

extern double g_camAngleHorizontal;
extern double g_camAngleVertical;
extern double g_camPitch;
extern int g_convex_pieces;
extern double g_angular_discretization_step;
extern koptError_t koptError;
extern double g_security_distance;

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

AGPSolver::AGPSolver(poly_t* p, int variables, int constraints)
:m_variables{variables}, m_constraints{constraints}, poly{*p}
{
    this->QPSolver = std::make_unique<QProblem>(QProblem(variables, constraints));
    this->orientationSolutionFound = false;
    this->solutionFound = false;
    this->currentIndexOfAMatrix = 0;
    this->currentIndexOfBoundMatrices = 0;
}

void AGPSolver::initPolygon(poly_t* p)
{
    poly_t& poly = *p;
    poly.numOfVertices = poly.vertices.size();
    int numOfVertices = poly.getnumOfVertices();
    Vector3f m(0, 0, 0);
    for(int i = 0; i < numOfVertices; i++)
    {
        m = m + poly.vertices[i];
    }
    m = m / numOfVertices;
    poly.centroid = m;

    if(numOfVertices == 3)
    {
        poly.a = ((poly.vertices[1] - poly.vertices[0]).cross(poly.vertices[2] - poly.vertices[1]))/2;
        poly.aabs = poly.a / poly.a.norm();
        for(int i = 0; i < numOfVertices; i++)
        {
            if(i == numOfVertices-1)
            {
                Vector3f q = poly.vertices[i-(numOfVertices-1)] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                Vector3f n_tmp = m * poly.a; n_tmp.normalize();
                poly.n.push_back(n_tmp);
            }
            else{
                Vector3f q = poly.vertices[i+1] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                Vector3f n_tmp = m * poly.a; n_tmp.normalize();
                poly.n.push_back(n_tmp);
            }
        }
    }
    else if(numOfVertices > 3)
    {
        poly.a = VID::Polygon::findPolyAreaVector(poly.vertices);
        poly.aabs = VID::Polygon::findUnitNormal(poly.vertices);
        // std::cout << poly.aabs << std::endl;
        for(int i = 0; i < numOfVertices; i++)
        {
            if(i == numOfVertices-1)
            {
                Vector3f q = poly.vertices[i-(numOfVertices-1)] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                Vector3f n_tmp = m * poly.a; n_tmp.normalize();
                poly.n.push_back(n_tmp);
            }
            else{
                Vector3f q = poly.vertices[i+1] - poly.vertices[i]; q.normalize();
                AngleAxisf m = AngleAxisf(poly.incidenceAngle, q);
                Vector3f n_tmp = m * poly.a; n_tmp.normalize();
                poly.n.push_back(n_tmp);
            }
        }
    }

}

AGPSolver::~AGPSolver()
{

}

void AGPSolver::calculateQProblemParams()
{
    poly.H = new real_t[this->m_variables * this->m_variables];
    poly.A = new real_t[this->m_constraints * this->m_variables];
    poly.d = new real_t[this->m_variables];
    poly.lbA = new real_t[this->m_constraints];
    poly.ubA = new real_t[this->m_constraints];

    // Initialize Gradient d
    poly.d[0] = 0.0; poly.d[1] = 0.0; poly.d[2] = 0.0;

    /* fill H matrix (Hessian Matrix)
    |4+2D    0    0|
    |   0 4+2D    0|
    |   0    0 4+2D|
    */
    for(int i=0; i < this->m_variables * this->m_variables; i++)
        poly.H[i] = 0.0;
    poly.H[0] = 4.0 + 2.0 * g_const_D; poly.H[4] = 4.0 + 2.0 * g_const_D; poly.H[8] = 4.0 + 2.0 * g_const_D;

    // Calculate A matrix, lbA and ubA before hand with all the constraints. 

    for(int i = 0; i<this->m_constraints * this->m_variables; i++)
        poly.A[i] = 0.0;
    
    
    this->setPositionConstraints(true); // Mandatory constraint
    this->setDminDmaxConstraint(true);

    Options options;
    this->QPSolver->setOptions( options );

}

void AGPSolver::setDminDmaxConstraint(bool flag)
{
    int i = currentIndexOfAMatrix;
    poly.A[i] = poly.aabs[0]; poly.A[i+1] = poly.aabs[1]; poly.A[i+2] = poly.aabs[2];

    int j = currentIndexOfBoundMatrices;
    // lba and uba
    poly.lbA[j] = poly.aabs.dot(poly.centroid+ poly.aabs * poly.minDist); 
    poly.ubA[j] = poly.aabs.dot(poly.centroid+ poly.aabs * poly.maxDist);
    currentIndexOfAMatrix = currentIndexOfAMatrix + 3;
    currentIndexOfBoundMatrices = currentIndexOfBoundMatrices + 1;
}

void AGPSolver::setPositionConstraints(bool flag)
{   
    int i = currentIndexOfAMatrix;
    int itr = 0;
    // poly.A[i] = poly.n[0][0]; poly.A[i+1] = poly.n[0][1]; poly.A[i+2] = poly.n[0][2];
    // poly.A[i+3] = poly.n[1][0]; poly.A[i+4] = poly.n[1][1]; poly.A[i+5] = poly.n[1][2];
    // poly.A[i+6] = poly.n[2][0]; poly.A[i+7] = poly.n[2][1]; poly.A[i+8] = poly.n[2][2];

    for( ; i < currentIndexOfAMatrix + this->m_variables * poly.vertices.size() && itr < poly.vertices.size(); i = i + 3, itr++)
    {
        poly.A[i] = poly.n[itr][0]; poly.A[i+1] = poly.n[itr][1]; poly.A[i+2] = poly.n[itr][2];
    } 

    int j = currentIndexOfBoundMatrices;
    itr = 0;
    // // lbA
    // poly.lbA[j] = poly.n[0].dot(poly.vertices[0]);
    // poly.lbA[j+1] = poly.n[1].dot(poly.vertices[1]);
    // poly.lbA[j+2] = poly.n[2].dot(poly.vertices[2]);
    // // ubA
    // poly.ubA[j] = FLT_MAX;
    // poly.ubA[j+1] = FLT_MAX;
    // poly.ubA[j+2] = FLT_MAX;

    for( ; j < currentIndexOfBoundMatrices + poly.vertices.size() && itr < poly.vertices.size(); j++)
    {
        poly.lbA[j] = poly.n[itr].dot(poly.vertices[itr]);
        poly.ubA[j] = FLT_MAX;
    }

    currentIndexOfAMatrix = currentIndexOfAMatrix + poly.vertices.size() * this->m_variables;
    currentIndexOfBoundMatrices = currentIndexOfBoundMatrices + poly.vertices.size();
}

void AGPSolver::setFOVConstraints(int pw, bool flag)
{
    double angleLower = g_camPitch + g_camAngleVertical/2.0;
    double angleUpper = g_camPitch - g_camAngleVertical/2.0;

    int maxPW = g_convex_pieces;
    double psiInc = 2*M_PI/maxPW;

    int numOfVertices = poly.getnumOfVertices();

    double psiD = pw*psiInc;
    Vector3f right(-sin(psiD-psiInc/2.0), cos(psiD-psiInc/2.0),0.0);
    Vector3f left(-sin(psiD+psiInc/2.0), cos(psiD+psiInc/2.0),0.0);
    Vector3f lowerCam(sin(angleLower)*cos(psiD), sin(angleLower)*sin(psiD), -cos(angleLower));
    Vector3f upperCam(-sin(angleUpper)*cos(psiD), -sin(angleUpper)*sin(psiD), cos(angleUpper));
    Vector3f low = poly.vertices[0];
    Vector3f high = poly.vertices[0];
    if(numOfVertices == 3)
    {
        if(low.dot(lowerCam)<poly.vertices[1].dot(lowerCam))
            low = poly.vertices[1];
        if(low.dot(lowerCam)<poly.vertices[2].dot(lowerCam))
            low = poly.vertices[2];
        
        
        if(high.dot(upperCam)<poly.vertices[1].dot(upperCam))
            high = poly.vertices[1];
        if(high.dot(upperCam)<poly.vertices[2].dot(upperCam))
            high = poly.vertices[2];
    }
    else if(numOfVertices > 3)
    {
        for(int i = 1; i < numOfVertices; i++)
        {
            if(low.dot(lowerCam)<poly.vertices[i].dot(lowerCam))
            {
                low = poly.vertices[i]; 
            }
        }
        for(int i = 1; i < numOfVertices; i++)
        {
            if(high.dot(upperCam)<poly.vertices[i].dot(upperCam))
            {
                low = poly.vertices[i]; 
            }
        }
    }
    

    int j = currentIndexOfBoundMatrices;
    /* fill elements 5-8 of lbA vector
    |          n_1^T*x_1        |
    |          n_2^T*x_2        |
    |          n_3^T*x_3        |
    |      a_N^T*x_1+d_min      |
    |        n_right^T*m        |
    |         n_left^T*m        |
    | n^cam_lower^T*x^rel_lower |
    | n^cam_upper^T*x^rel_upper |
    */
    poly.lbA[j] = right.dot(poly.centroid); 
    poly.lbA[j+1] = left.dot(poly.centroid); 
    poly.lbA[j+2] = lowerCam.dot(low); 
    poly.lbA[j+3] = upperCam.dot(high); 
    /* fill elements 5-8 of ubA vector
    |            inf            |
    |            inf            |
    |            inf            |
    |      a_N^T*x_1+d_max      |
    |            inf            |
    |            inf            |
    |            inf            |
    |            inf            |
    */
    poly.ubA[j] = FLT_MAX; 
    poly.ubA[j+1] = FLT_MAX;
    poly.ubA[j+2] = FLT_MAX;
    poly.ubA[j+3] = FLT_MAX; 

    currentIndexOfBoundMatrices = currentIndexOfBoundMatrices + 4;

    int i = currentIndexOfAMatrix;
    /* fill lines 5-8 of A matrix
    |[      n_1^T      ]|
    |[      n_2^T      ]|
    |[      n_3^T      ]|
    |[      a_N^T      ]|
    |[    n_right^T    ]|
    |[     n_left^T    ]|
    |[  n^cam_lower^T  ]|
    |[  n^cam_upper^T  ]|
    */
    poly.A[i] = right[0]; 
    poly.A[i+1] = right[1]; 
    poly.A[i+2] = right[2]; 
    poly.A[i+3] = left[0]; 
    poly.A[i+4] = left[1];
    poly.A[i+5] = left[2];

    poly.A[i+6] = lowerCam[0];
    poly.A[i+7] = lowerCam[1]; 
    poly.A[i+8] = lowerCam[2]; 
    poly.A[i+9] = upperCam[0]; 
    poly.A[i+10] = upperCam[1]; 
    poly.A[i+11] = upperCam[2];

    currentIndexOfAMatrix = currentIndexOfAMatrix + 12;
}

std::tuple<StateVector, int> AGPSolver::findViewPointSolution(StateVector* state1, StateVector* state2, StateVector* statePrev)
{
    double DD = 0.5 * (poly.minDist + poly.maxDist);
    double cost = DBL_MAX;
    assert(DD < poly.maxDist);
    StateVector best;
    for(int i = 0; i < DIMENSIONALITY; i++){
        best[i] = 0;
    }

    double angleLower = g_camPitch + g_camAngleVertical/2.0;
    double angleUpper = g_camPitch - g_camAngleVertical/2.0;

    int maxPW = g_convex_pieces;
    double psiInc = 2*M_PI/maxPW;
    int obsCount = 0;
    for(int pw=0; pw < maxPW; pw++){
        
        this->setFOVConstraints(pw, true);

        currentIndexOfAMatrix = currentIndexOfAMatrix - 3*4;
        currentIndexOfBoundMatrices = currentIndexOfBoundMatrices - 4;

        // State vector g for the initial iteration
        StateVector g;
        g << poly.centroid[0] + DD * poly.aabs[0],
             poly.centroid[1] + DD * poly.aabs[1],
             poly.centroid[2] + DD * poly.aabs[2], 0.0;
             
        static real_t lbx[3] = { X_MIN, Y_MIN, Z_MIN };
        static real_t ubx[3] = { X_MAX, Y_MAX, Z_MAX };
        int nWSR = 100;
        returnValue re;
        bool solFoundLocal = false;
        /* fill vector d
        ||          |   |           |   |           ||
        || -2Dg^k-1 | + | -2g_p^k-1 | + | -2g_s^k-1 ||
        ||          |   |           |   |           ||
        */

        poly.d[0] = -(4.0 + 2.0*g_const_D) * g[0];
        poly.d[1] = -(4.0 + 2.0*g_const_D) * g[1];
        poly.d[2] = -(4.0 + 2.0*g_const_D) * g[2];
        if(SUCCESSFUL_RETURN != (re = this->QPSolver->init(poly.H, poly.d, poly.A, lbx, ubx, poly.lbA, poly.ubA, nWSR)))
        {
            // ROS_INFO("Return Value: %d", re);
            continue;
        }
        else
        {
            solFoundLocal = true;
        }

        /* compute the constant term of the optimization objective
        g_p^k-1^T*g_p^k-1 + g_s^k-1^T*g_s^k-1 + D*g^k-1^T*g^k-1
        */
        double xxCompensate = 0.0;

        for(int k = 0; k < 3; k++)
            xxCompensate += pow(g[k],2.0);
        xxCompensate *= 4.0 + 2.0*g_const_D;

        real_t xOptPosition[3]; // To store the optimum position obtained by the solver.
        this->QPSolver->getPrimalSolution(xOptPosition);
        
        g[0] = xOptPosition[0];
        g[1] = xOptPosition[1];
        g[2] = xOptPosition[2];


        // Find suitable orientation for the position.
        double costOrientation = this->findOrientationSolution(g, state1, state2);

        if(this->QPSolver->getObjVal() + xxCompensate + costOrientation < cost && solFoundLocal)
        {
            best = g;
            cost = this->QPSolver->getObjVal() + xxCompensate + costOrientation;
        }
        solutionFound |= solFoundLocal;


    }

    return std::make_tuple(best, obsCount);

}

double AGPSolver::findOrientationSolution(StateVector& g, StateVector* state1, StateVector* state2)
{
    double costOrientation = DBL_MAX;
    double dp = 1e-9;
    double ds = 1e-9;
    double alfa1 = 0.0;
    double alfa2 = 0.0;

    float maxArea = 0.0;
    bool isPosSame = false;

    for(double psi = -M_PI; psi < M_PI; psi += g_angular_discretization_step)
    {
        StateVector s = g;
        s[3] = psi;
        double c = 0.9 * DBL_MAX;

        // Max Screen area
        float area = 0.0;
        std::vector<Vector2f> verticesOnScreen = locateVerticesOnScreen(this->poly.vertices, Vector3f(s[0], s[1], s[2]), (float)psi);
        area = findAreaOfPolyOnScreen(verticesOnScreen);
        if(c <= costOrientation && this->isVisible(s) /*&& area >= maxArea*/)
        {
            g[3] = s[3];
            costOrientation = c; 
            maxArea = area;
            this->orientationSolutionFound = true;
        }

    }
    // g = movePositionTowardsPolygon(g);
    // Move further towards the polygon
    // int i = 1;
    // while(!isPosSame)
    // {
        
    //     StateVector temp = movePositionTowardsPolygon(g);
    //     if(temp == g)
    //     {
    //         isPosSame = true;
    //         g = temp;
    //         ROS_INFO("Pos adjustment iterations: %d", i);
    //         break;
    //     }
    //     else
    //     {
    //         g = temp;
    //         isPosSame = false;
    //     }
    //     i++;
    // }
    
    // std::cout << "Max Area: " << maxArea << std::endl;
    return costOrientation;
}

bool AGPSolver::isVisible(StateVector s)
{
    Vector3f st(s[0], s[1], s[2]);
    auto vertices = poly.getVertices();
    if(poly.aabs.dot(st-poly.centroid-poly.aabs*poly.minDist)<0)
        return false;
    if(poly.aabs.dot(st-poly.centroid-poly.aabs*poly.maxDist)>0)
        return false;

    // if((st - vertices[0]).dot(poly.n[0]) < 0)
    //     return false;
    // if((st - vertices[1]).dot(poly.n[1]) < 0)
    //     return false;
    // if((st - vertices[2]).dot(poly.n[2]) < 0)
    //     return false;
    
    // for(int it = 0; it<  VID::Polygon::camBoundNormal.size(); it++) {
    //     Vector3f camN = camBoundRotated( VID::Polygon::camBoundNormal[it], 0.0, s[3]);
    //     if(camN.dot(vertices[0] - st) < 0)
    //         return false;
    //     if(camN.dot(vertices[1] - st) < 0)
    //         return false;   
    //     if(camN.dot(vertices[2] - st) < 0)
    //         return false; 
    // }

    int numOfVertices = poly.getnumOfVertices();
    for(int i = 0; i < numOfVertices; i++)
    {
        if((st - vertices[i]).dot(poly.n[i]) < 0)
            return false;
    }

    for(int it = 0; it<  VID::Polygon::camBoundNormal.size(); it++) {
        Vector3f camN = camBoundRotated( VID::Polygon::camBoundNormal[it], 0.0, s[3]);
        for(int i = 0; i < numOfVertices; i++)
        {
            if(camN.dot(vertices[i] - st)<0)
                return false;
        }
    }
    return !this->IsInCollision(s);
}

bool AGPSolver::IsInCollision(StateVector s)
{
    for(typename std::list<VID::region*>::iterator iter = System::obstacles.begin(); iter != System::obstacles.end(); iter++)
    {
        if(fabs((*iter)->center[0] - s[0]) < (*iter)->size[0]/2.0 + g_security_distance &&
            fabs((*iter)->center[1] - s[1]) < (*iter)->size[1]/2.0 + g_security_distance &&
            fabs((*iter)->center[2] - s[2]) < (*iter)->size[2]/2.0 + g_security_distance)
        {
            return true;
        }


    }
    return false;
}

Vector3f AGPSolver::camBoundRotated(Vector3f normal, double roll, double yaw)
{
    Vector3f x(1, 0, 0);
    Vector3f z(0, 0, 1);
    AngleAxisf mroll = AngleAxisf(roll, x);
    AngleAxisf myaw = AngleAxisf(yaw, z);
    return myaw*(mroll*normal);
}

StateVector AGPSolver::dualBarrierSamplerFresh(StateVector* state1, StateVector* state2, StateVector* statePrev)
{
    if(poly.Fixpoint){
        StateVector ret;
        for(int i = 0; i<3; i++)
        {
            ret[i] = poly.vertices[0][i];
        }
        ret[3] = poly.vertices[1][0];
        return ret;
    }
    
    // Calculate initial parameters for QProblem solver.
    this->calculateQProblemParams();

    // View point Solution method
    StateVector best; 
    int obsCount;
    std::tie(best, obsCount) = this->findViewPointSolution(state1, state2, statePrev);
    if(obsCount >= g_convex_pieces)
    {
        std::string pkgPath = ros::package::getPath("planner_node");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
        plannerLog << "-->Too may obstacles in the dual sampling space!\n";
        for(int i = 0; i < poly.getnumOfVertices(); i++)
        {
            plannerLog << "   x" << i+1 << ": (" << poly.vertices[i][0] << ", " << poly.vertices[i][1] 
                        << ", " << poly.vertices[i][2] << ")\n";
        }
        plannerLog.close();
        koptError = OBSTACLE_INFEASIBILITY;
    }

    if(!solutionFound)
    {

        std::string pkgPath = ros::package::getPath("planner_node");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
        plannerLog << "-->No feasible position to inspect polygon:\n";
        for(int i = 0; i < poly.getnumOfVertices(); i++)
        {
            plannerLog << "   x" << i+1 << ": (" << poly.vertices[i][0] << ", " << poly.vertices[i][1] 
                        << ", " << poly.vertices[i][2] << ")\n";
        }
        plannerLog.close();
        koptError = VIEWPOINT_INFEASIBILITY;
    }

    if(!orientationSolutionFound)
    {
      
        std::string pkgPath = ros::package::getPath("planner_node");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
        plannerLog << "-->No feasible heading to inspect polygon:\n";
        for(int i = 0; i < poly.getnumOfVertices(); i++)
        {
            plannerLog << "   x" << i+1 << ": (" << poly.vertices[i][0] << ", " << poly.vertices[i][1] 
                        << ", " << poly.vertices[i][2] << ")\n";
        }
        plannerLog.close();
        koptError = VIEWPOINT_INFEASIBILITY;      
    }
    for(int i = 0; i < 4; i++)
        assert(best[i] < 1e15 && best[i] > -1e15);
    
    ROS_INFO("Sampled VP: {x, y, z, yaw} = {%f, %f, %f, %f}", best[0], best[1],best[2], best[3]);
    return best;

}

Matrix3f AGPSolver::cameraMtx;

void AGPSolver::setCameraMtx(std::string node)
{
    std::vector<float> K;
    ros::param::get(node + "/camera/K", K);
    AGPSolver::cameraMtx << K[0], K[1], K[2],
                            K[3], K[4], K[5],
                            K[6], K[7], K[8];
}

std::vector<Vector2f> AGPSolver::locateVerticesOnScreen(std::vector<Vector3f> vertices, Vector3f posInWorld, float yaw)
{
    Matrix3f R_w2c;
    R_w2c << 0, -1.0, 0,
             0, 0, -1.0,
             1.0, 0, 0;

    Matrix3f R = Matrix3f::Identity();
    
    // Yaw rot matrix
    AngleAxis rollAngle_y((float)0.0, Vector3f::UnitX());
    AngleAxis pitchAngle_y((float)0.0, Vector3f::UnitY());
    AngleAxis yawAngle_y((float)yaw, Vector3f::UnitZ());
    Quaternion<float> q_yaw = rollAngle_y * pitchAngle_y * yawAngle_y;
    Matrix3f R_yaw = q_yaw.matrix();

    // Pitch rot matrix
    AngleAxis rollAngle_p((float)0.0, Vector3f::UnitZ());
    AngleAxis pitchAngle_p((float)(-g_camPitch), Vector3f::UnitX());
    AngleAxis yawAngle_p((float)0.0, Vector3f::UnitY());
    Quaternion<float> q_pitch = rollAngle_p * pitchAngle_p * yawAngle_p;
    Matrix3f R_pitch = q_pitch.matrix();

    auto neg_Rt = -R * posInWorld;
    auto R_prime = R_pitch * R_w2c * R_yaw;

    auto t = R_prime * neg_Rt;

    // std::cout << t.transpose() << std::endl;


    Matrix3f R_final = R_prime * R;
    Matrix4f T_w2c;
    T_w2c <<  R_final(0,0), R_final(0,1),R_final(0,2), t[0],
              R_final(1,0), R_final(1,1), R_final(1,2), t[1],
              R_final(2,0), R_final(2,1), R_final(2,2), t[2],
              0, 0, 0, 1;
    // std::cout << T_w2c << std::endl;
    std::vector<Vector3f> verticesInCC; // Vertices in Camera coordinate system.
    for(auto& v : vertices)
    {
        Vector4f vertexHomogenous(v[0], v[1], v[2], 1);
        Vector4f vertexInCC = T_w2c * vertexHomogenous;
        verticesInCC.push_back(Vector3f(vertexInCC[0], vertexInCC[1], vertexInCC[2]));
        // std::cout << "VertexInCC: " << vertexInCC.transpose() << std::endl;
    }

    std::vector<Vector2f> verticesOnScreen;
    for(auto& ver : verticesInCC)
    {
        Vector3f vertexOnScreen = cameraMtx * ver;
        float Zc = vertexOnScreen[2];
        float u = vertexOnScreen[0] / Zc;
        float v = vertexOnScreen[1] / Zc;
        verticesOnScreen.push_back(Vector2f(u, v));
    }

    return verticesOnScreen;
}

float AGPSolver::findAreaOfPolyOnScreen(std::vector<Vector2f>& verOnScreen)
{
    std::vector<Vector2f> v = verOnScreen;
    int N = verOnScreen.size();
    float area = 0.0;
    // for(auto v :verOnScreen)
    // {
    //     std::cout << "Vertex: " << v.transpose() << std::endl;
    // }
    if(N >= 3)
    {
        float S_poly = 0.0;
        for(int i = 0; i < N; i++)
        {
            S_poly += (v[i][0] * v[(i+1) % N][1] - v[i][1] * v[(i+1) % N][0]);
        }
        area = abs(0.5 * S_poly);
    }
    else
    {
        ROS_ERROR("OnScreen: Not a 2D shape, therefore, no area!");
    }
    
    return area;
}

StateVector AGPSolver::movePositionTowardsPolygon(StateVector& g)
{
    Vector3f pos(g[0], g[1], g[2]);
    float ori = g[3];
    bool isVisible = true; 

    Vector3f distVec =  poly.centroid - pos; 
    // std::cout << "distVec : " << distVec.transpose() << std::endl;
    Vector3f distVecNorm = distVec / distVec.norm();
    // std::cout << "distVecNorm : " << distVecNorm.transpose() << std::endl;

    Vector3f posPrime = pos;

    posPrime[0] += (distVecNorm[0] * 1.0);
    posPrime[1] += (distVecNorm[1] * 1.0);
    posPrime[2] += (distVecNorm[2] * 1.0);
    isVisible = false;
    double costOrientation = DBL_MAX;
    float maxArea = 0.0;
    for(double psi = -M_PI; psi < M_PI; psi += g_angular_discretization_step)
    {
        StateVector s(posPrime[0], posPrime[1], posPrime[2], psi);
        double c = 0.9 * DBL_MAX;

        // Max Screen area
        float area = 0.0;
        std::vector<Vector2f> verticesOnScreen = locateVerticesOnScreen(this->poly.vertices, Vector3f(s[0], s[1], s[2]), (float)psi);
        area = findAreaOfPolyOnScreen(verticesOnScreen);
        
        if(c <= costOrientation && this->isVisible(s) && area >= maxArea)
        {
            ori = s[3];
            pos = posPrime;
            costOrientation = c; 
            maxArea = area;
        }
    }
    isVisible = this->isVisible(StateVector(pos[0], pos[1], pos[2], ori));
        

    return StateVector(pos[0], pos[1], pos[2], ori);

}