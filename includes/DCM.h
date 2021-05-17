#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "cmath"
#include "fstream"
#include "iostream"
#include "math.h"
#include <vector>

#include "Trajectory.h"


using namespace Eigen;
using namespace std;

#define G 9.807

class DCMPlanner : public TrajectoryPlanner{
    public:
        DCMPlanner(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double deltaZ);
        void setRVRP();
    private:
        vector<Vector3d> rF_;
        double dt_;
        double tStep_;
        double tDS_;
        double alpha_;
        double deltaZ_;
};