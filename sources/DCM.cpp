#include "../includes/DCM.h"

DCMPlanner::DCMPlanner(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double deltaZ){
    rF_ = rF;
    tStep_ = tStep;
    tDS_ = tDS;
    alpha_ = alpha;
    deltaZ_ = deltaZ;
    dt_ = dt;
    footCount_ = rF.size();
}

void DCMPlanner::setRVRP(){
    Vector3d z_vrp(0.0, 0.0, deltaZ_);
    rVRP_.resize(footCount_-1);
    for(int i=1; i<footCount_; i++){
        rVRP_[i-1] = rF_[i] + z_vrp;
    }
}

void DCMPlanner::setXiEOS(){
    XiEOS_.resize(footCount_-2);
    XiEOS_[footCount_-3] = rVRP_[footCount_-2];
    for(int i=footCount_-4; i>=0; i--){
        XiEOS_[i] = rVRP_[i+1] + exp(-sqrt(G / deltaZ_) * tStep_) * (XiEOS_[i+1]-rVRP_[i+1]);
    }
}
void DCMPlanner::setInitXi(){
    int length = (1/dt_)*tStep_*(footCount_-2);
    InitXi_.resize(length);
    for(int i=0; i<length; i++){
        double local_t = fmod((i*dt_), tStep_);
        int step_num = floor(i*dt_/tStep_);
        InitXi_[i] = rVRP_[step_num] + exp(sqrt(G / deltaZ_)*(local_t - tStep_))*(XiEOS_[step_num]-rVRP_[step_num]);
    }
}

void DCMPlanner::findDSBC(){
    InitXiDS_.resize(footCount_-2);
    InitXiDDS_.resize(footCount_-2);
    EndXiDS_.resize(footCount_-2);
    EndXiDDS_.resize(footCount_-2);
    for(int i=0; i<footCount_-2; i++){
        if(i==0){
            InitXiDS_[i] = InitXi_[i];
            EndXiDS_[i] = rVRP_[i] + exp(sqrt(G / deltaZ_)*(tDS_)*(1-alpha_))*(InitXi_[i] - rVRP_[i]);
            InitXiDDS_[i] = (InitXiDS_[i] - rVRP_[i])*sqrt(G / deltaZ_);
            EndXiDDS_[i] = (EndXiDS_[i] - rVRP_[i])*sqrt(G / deltaZ_);
        }else{
            InitXiDS_[i] = rVRP_[i-1] + exp(-sqrt(G / deltaZ_)*(tDS_)*(alpha_))*(XiEOS_[i-1] - rVRP_[i-1]);
            EndXiDS_[i] = rVRP_[i] + exp(sqrt(G / deltaZ_)*(tDS_)*(1-alpha_))*(XiEOS_[i-1] - rVRP_[i]);
            InitXiDDS_[i] = (InitXiDS_[i] - rVRP_[i-1])*sqrt(G / deltaZ_);
            EndXiDDS_[i] = (EndXiDS_[i] - rVRP_[i])*sqrt(G / deltaZ_);
        }
    }
}

void DCMPlanner::phaseDetect(){
    InitDSt_.resize(footCount_-2);
    EndDSt_.resize(footCount_-2);
    InitSSt_.resize(footCount_-2);
    EndSSt_.resize(footCount_-2);
    InitDSt_[0] = 0.0;
    EndDSt_[0] = tDS_*1/dt_ - 1;
    InitSSt_[0] = EndDSt_[0] + 1;
    for(int i=1; i<footCount_ - 2; i++){
        InitDSt_[i] = (i*tStep_ - tDS_*alpha_)*1/dt_ ;
        EndDSt_ [i] = (i*tStep_ + tDS_*(1-alpha_))*1/dt_ - 1;
        EndSSt_[i-1] = InitDSt_[i] - 1;
        InitSSt_[i] = EndDSt_ [i] + 1;
    }
    EndSSt_[footCount_ - 3] = InitSSt_[footCount_ - 3] + (tStep_ - tDS_) * 1/dt_ - 1;
}

void DCMPlanner::mergeXi(){
    Xi_ = InitXi_;
    for(int i=0; i<footCount_ - 2; i++){
        vector<Vector3d> way_pts = {InitXiDS_[i], EndXiDS_[i]};
        vector<Vector3d> vel_pts = {InitXiDDS_[i], EndXiDDS_[i]};
        double time_pts[] = {0, tDS_};
        vector<Vector3d> xi_ds = TrajectoryPlanner::cubicPoly(way_pts, vel_pts, time_pts, dt_);
        for(int j=InitDSt_[i]; j<=EndDSt_ [i]; j++){
            Xi_[j] = xi_ds[j - InitDSt_[i]];
        }
    }
    this->write2File(Xi_ ,"Xi");
}

void DCMPlanner::write2File(vector<Vector3d> input ,string file_name="data"){
    ofstream output_file("files/" + file_name + ".csv");
    int size = input.size();
    for(int i=0; i<size; i++){
         output_file << input[i](0) << " ,";
         output_file << input[i](1) << " ,";
         output_file << input[i](2) << " ,";
         output_file << "\n";
    }
    output_file.close();
}

void DCMPlanner::setCOM(){
    int length = (1/dt_)*tStep_*(footCount_-2);
    COM_.resize(length);
    COMD_.resize(length);
    COM_[0] << 0.0, 0.0, deltaZ_;
    COMD_[0] << 0.0, 0.0, 0.0;
    for(int i=1; i<length; i++){
        Vector3d inte(0.0, 0.0, 0.0);
       
        for(int j=0; j <= i; j++){
            inte += dt_ * Xi_[j]* exp((j * dt_)/(sqrt(deltaZ_/G)));
        }
        
        COM_[i] = ((inte / (sqrt(deltaZ_/G))) + COM_[0])*exp((-1*i * dt_)/(sqrt(deltaZ_/G))) ;
        COMD_[i] = (sqrt(G/deltaZ_)) * (Xi_[i] - COM_[i]);

    }
    this->write2File(COM_ ,"COM");
}

vector<Vector3d> DCMPlanner::getCOM(){
    this->setRVRP();
    this->setXiEOS();
    this->setInitXi();
    this->findDSBC();
    this->phaseDetect();
    this->mergeXi();
    this->setCOM();
    return COM_;
}

// int main(){
//     vector<Vector3d> v1(9);
//     v1[0] << 0.0, 0.12, 0.0;
//     v1[1] << 0.0, -0.12, 0.0;
//     v1[2] << 0.5, 0.12, 0.0;
//     v1[3] << 1.0, -0.12, 0.0;
//     v1[4] << 1.5, 0.12, 0.0;
//     v1[5] << 2.0, -0.12, 0.0;
//     v1[6] << 2.5, 0.12, 0.0;
//     v1[7] << 3.0, -0.12, 0.0;
//     v1[8] << 3.0, 0.12, 0.0;
//     DCMPlanner d1(v1, 0.01, 0.8, 0.3, 0.3, 0.68);
//     d1.setRVRP();
//     d1.setXiEOS();
//     d1.setInitXi();
//     d1.findDSBC();
//     d1.phaseDetect();
//     d1.mergeXi();
//     d1.setCOM();
//     return 0;
// }