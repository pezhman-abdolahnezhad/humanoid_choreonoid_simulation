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
        AnklePlanner(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double swingHeight);
        void phaseDetect();
        void setDSTraj();
        void setSSTraj();
        void write2File(vector<Vector3d> input ,string file_name);
        vector <vector<Vector3d>> getAnkles();


    private:
        vector<Vector3d> rF_;
        double dt_;
        double tStep_;
        double tDS_;
        double alpha_;
        double swingHeight_;
        int footCount_;

        vector <Vector3d> q1_;
        vector <Vector3d> q2_;

        vector <double> InitDSt_;
        vector <double> EndDSt_;
        vector <double> InitSSt_;
        vector <double> EndSSt_;
};