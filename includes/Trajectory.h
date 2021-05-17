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
        TrajectoryPlanner(double dt);
        MatrixXd cubicPoly(double* way_pts, double* vel_pts, double* time_pts, const int pts_count);
        double* cubicCoefs(double theta_ini, double theta_f, double theta_dot_ini, double theta_dot_f, double tf);
    protected:
        double dt_;
};