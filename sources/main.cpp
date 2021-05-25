#include <cnoid/SimpleController>
#include <vector>
#include "Robot.h"
using namespace cnoid;

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

class SurenaV : public SimpleController
{
    BodyPtr ioBody;
    double dt;
    std::vector<double> qref;
    std::vector<double> qold;
    vector<vector<double>> RJangles_;
    vector<vector<double>> LJangles_;
    int idx = 0;

public:
    void setJntAng(){
        Robot surena();
        vector<Vector3d> rF(7);
        rF[0] << 0.0, 0.09, 0.0;
        rF[1] << 0.0, -0.09, 0.0;
        rF[2] << 0.5, 0.09, 0.0;
        rF[3] << 1.0, -0.09, 0.0;
        rF[4] << 1.5, 0.09, 0.0;
        rF[5] << 2.0, -0.09, 0.0;
        rF[6] << 2.0, 0.09, 0.0;

        Robot d1(rF, 0.001, 0.8, 0.3, 0.3, 0.69, 0.1, 0.09, 0.3535, 0.36);
        //cout << "Haaaaa";
        vector<vector<vector<double>>> jnt_angs = d1.getJntAng();
        RJangles_ = jnt_angs[0];
        LJangles_ = jnt_angs[1];
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->setJntAng();
        //cout << LJangles_[0][2]<< endl;
        ioBody = io->body();
        dt = io->timeStep();

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
    
        qold = qref;


        return true;
    }

    virtual bool control() override
    {
        for(int i=0; i<6; i++){
            qref[i] = RJangles_[idx][i];
            qref[13 + i] = LJangles_[idx][i];
        }
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
            joint->u() = u;
        }
        if(idx<3900){
            idx ++;
        }
            
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SurenaV)
