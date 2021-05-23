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
        void setXiEOS();
        void setInitXi();
        void findDSBC();  //finds double support boundary conditions
        //void setXiDS();
        void phaseDetect(); //detects different walking phases index(SS & DS)
        void mergeXi();
        void write2File(vector<Vector3d> input ,string file_name);
        void setCOM();
        vector<Vector3d> getCOM();

    private:
        vector<Vector3d> rF_;
        vector<Vector3d> rVRP_;
        vector<Vector3d> XiEOS_;
        vector<Vector3d> InitXi_;
        vector<Vector3d> InitXiDS_;
        vector<Vector3d> InitXiDDS_;
        vector<Vector3d> EndXiDS_;
        vector<Vector3d> EndXiDDS_;
        vector<Vector3d> Xi_;
        vector<Vector3d> COM_;
        vector<Vector3d> COMD_; //COM Velocity

        double dt_;
        double tStep_;
        double tDS_;
        double alpha_;
        double deltaZ_;
        int footCount_;

        vector <double> InitDSt_;
        vector <double> EndDSt_;
        vector <double> InitSSt_;
        vector <double> EndSSt_;

};