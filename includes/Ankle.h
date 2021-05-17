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


class AnklePlanner : public TrajectoryPlanner{
    public:
        AnklePlanner(Vector3d* initFP, Vector3d* finFP, double* initT, double* finT, double swingHeight, int swingCount, double dt);
        MatrixXd* getSwing();
    private:
        Vector3d* initFP_;   // swing foots initial points
        Vector3d* finFP_;    // swing foots final points
        double* initT_;      // initial times for swing
        double* finT_;       // final times for swing
        int swingCount_;
        double swingHeight_;
};