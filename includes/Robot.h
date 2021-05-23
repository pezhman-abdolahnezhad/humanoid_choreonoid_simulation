#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "cmath"
#include "fstream"
#include "iostream"
#include "math.h"
#include <vector>

#include "DCM.h"
#include "Ankle.h"

using namespace Eigen;
using namespace std;


class Robot{    
    public:
        Robot(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double deltaZ, double swingHeight, double D, double A, double B);
        void getTrajs();
        vector<double>  geometricalIK(Vector3d rel_ankle ,double D);
        int sign(double v);
        MatrixXd RPitch(double theta);
        MatrixXd RRoll(double phi);
        void calcJntAng();    // calculation of joint angles
        void write2File(vector<vector<double>> input ,string file_name);


    
    
    
    private:
        vector<Vector3d> rF_;
        double dt_;
        double tStep_;
        double tDS_;
        double alpha_;
        double deltaZ_;
        double swingHeight_;
        double D_;
        double A_;
        double B_;

        bool isLeft_;
        vector<Vector3d> COM_;
        vector<Vector3d> RFoot_;
        vector<Vector3d> LFoot_;
        vector<vector<double>> RJangles_;
        vector<vector<double>> LJangles_;
        

};