#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "cmath"
#include "fstream"
#include "iostream"
#include "math.h"
#include <vector>


using namespace Eigen;
using namespace std;


class TrajectoryPlanner{
    
    public:
        vector<Vector3d> cubicPoly(vector<Vector3d>& way_pts,vector<Vector3d>& vel_pts, double* time_pts, double dt);
        vector<Vector3d> cubicCoefs(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
};