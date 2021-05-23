//#include "../includes/Robot.h"
#include "Robot.h"
Robot::Robot(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double deltaZ, double swingHeight, double D, double A, double B){
    rF_ = rF;
    tStep_ = tStep;
    tDS_ = tDS;
    alpha_ = alpha;
    deltaZ_ = deltaZ;
    dt_ = dt;
    swingHeight_ = swingHeight;
    D_ = D;
    A_ = A;
    B_ = B;

    if(rF_[2](2)>0) isLeft_ = true;
    else isLeft_ = false;
}


 void Robot::getTrajs(){
    DCMPlanner d1(rF_, dt_, tStep_, tDS_, alpha_, deltaZ_);
    COM_ = d1.getCOM();
    
    
    AnklePlanner a1(rF_, dt_, tStep_, tDS_, alpha_, swingHeight_);
    vector<vector<Vector3d>> tot_q = a1.getAnkles();
    if(isLeft_){
        LFoot_ = tot_q[0];
        RFoot_ = tot_q[1];
    }else{
        LFoot_ = tot_q[1];
        RFoot_ = tot_q[0];
    }

 }
 int Robot::sign(double v){
  return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}
MatrixXd Robot::RPitch(double theta){
    MatrixXd Ry(3,3);
    double c=cos(theta);
    double s=sin(theta);
    Ry<<c,0,s,0,1,0,-1*s,0,c;
    return Ry;
}

MatrixXd Robot::RRoll(double phi){
    MatrixXd R(3,3);
    double c=cos(phi);
    double s=sin(phi);
    R<<1,0,0,0,c,-1*s,0,s,c;
    return R;
}

vector<double> Robot::geometricalIK(Vector3d rel_ankle, double D){
    Vector3d rel_hip = {0.0, D, 0.0};
    Vector3d r = rel_hip - rel_ankle;
    double C = r.norm();
    double c5 = (C*C-A_*A_ - B_*B_)/(2*A_*B_);
    double q2,q3,q4,q5,q6,q6a,q7;
    if (c5>=1) {
        q5=0;
        // cout<<"c5 is larger than 1"<<A<<","<<B<<","<<C<<","<<D<<","<<E<<"\n";
        cout<<"c5="<<c5<<"\n";
    }else if (c5<=-1){
        q5=M_PI;
    }else{
        q5=acos(c5); //Knee Pitch
    }
    q6a = asin((A_/C)*sin(M_PI-q5));//Ankle Pitch
    q7 = atan2(r(1),r(2));//Ankle Roll
    if (q7>1*M_PI/2){
        q7 = q7 - M_PI;
    }else if (q7<-1*M_PI/2) {
        q7 = q7 + M_PI;
    }
    double x = r(0);
    double y = sign(r(2))*sqrt(pow(r(1),2)+pow(r(2),2));
    q6 = -1*atan2(x,y)-q6a;//ankle pitch

    MatrixXd Rpitch=RPitch(-1*q5-1*q6);
    MatrixXd Rroll=RRoll(-1*q7);
    MatrixXd R = Rroll*Rpitch; // hipZ*hipX*hipY
    q2=atan2(-1*R(0,1),R(1,1));
    double cz=cos(q2);
    double sz=sin(q2);
    q3=atan2(R(2,1),-1*R(0,1)*sz+R(1,1)*cz);
    q4=atan2(-1*R(2,0),R(2,2));
    vector<double> q = {q2,q3,q4,q5,q6,q7};
    return q;  
}
void Robot::calcJntAng(){
    Vector3d rel_pos;
    RJangles_.resize(COM_.size());
    LJangles_.resize(COM_.size());
    for(int i=0; i<COM_.size(); i++){
        rel_pos = COM_[i] - LFoot_[i];
        LJangles_[i] = this->geometricalIK(rel_pos, D_);
        rel_pos = COM_[i] - RFoot_[i];
        RJangles_[i] = this->geometricalIK(rel_pos, -D_);
    }
    this->write2File (LJangles_, "LJangles");
    this->write2File (RJangles_, "RJangles");
}
void Robot::write2File(vector<vector<double>> input ,string file_name="data"){
    ofstream output_file("files/" + file_name + ".csv");
    int size = input.size();
    for(int i=0; i<size; i++){
        for(int j=0; j<input[0].size(); j++){
            output_file << input[i][j]*180/M_PI << " ,";
        } 
        output_file << "\n";
    }
    output_file.close();
}
//  int main(){
//     vector<Vector3d> v1(9);
//     v1[0] << 0.0, 0.115, 0.0;
//     v1[1] << 0.0, -0.115, 0.0;
//     v1[2] << 0.3, 0.115, 0.0;
//     v1[3] << 0.6, -0.115, 0.0;
//     v1[4] << 0.9, 0.115, 0.0;
//     v1[5] << 1.2, -0.115, 0.0;
//     v1[6] << 1.5, 0.115, 0.0;
//     v1[7] << 1.8, -0.115, 0.0;
//     v1[8] << 1.8, 0.115, 0.0;
//     Robot d1(v1, 0.01, 0.8, 0.3, 0.3, 0.68, 0.1, 0.115, 0.37, 0.36);
//     d1.getTrajs();
//     d1.calcJntAng();
//     // vector<double> q = d1.geometricalIK(-0.115, 0.37, 0.36, Vector3d(0.0, -0.115, -0.73));
//     return 0;
// }