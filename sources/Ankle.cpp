#include "../includes/Ankle.h"

AnklePlanner::AnklePlanner(vector<Vector3d> rF, double dt, double tStep, double tDS, double alpha, double swingHeight){
    rF_ = rF;
    dt_ = dt;
    tStep_ = tStep;
    tDS_ = tDS;
    alpha_ = alpha;
    swingHeight_ = swingHeight;
    footCount_ = rF.size();
}

void AnklePlanner::phaseDetect(){
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


void AnklePlanner::setDSTraj(){
    int length = (1/dt_)*tStep_*(footCount_-2);
    q1_.resize(length);
    q2_.resize(length);
    for(int i=0; i<InitDSt_.size(); i++){
        for(int j=InitDSt_[i]; j<=EndDSt_ [i]; j++){
            if(i%2 == 0){
                q1_[j] = rF_[i];
                q2_[j] = rF_[i+1];
            }else{
                q1_[j] = rF_[i+1];
                q2_[j] = rF_[i];
            }  
        }
    }
}
void AnklePlanner::setSSTraj(){
    for(int i=0; i<InitSSt_.size(); i++){

        Vector3d mid_pt((rF_[i](0) + rF_[i+2](0))/2, rF_[i](1), swingHeight_);
        double t_ss = (EndSSt_[i] - InitSSt_[i])*(dt_) + dt_;
        Vector3d vel_mid_pt((rF_[i+2](0) - rF_[i](0))/t_ss , 0.0, 0.0);
        vector<Vector3d> way_pts = {rF_[i], mid_pt, rF_[i+2]};
        vector<Vector3d> vel_pts(3); 
        vel_pts[0] << 0.0, 0.0, 0.0;
        vel_pts[1] = vel_mid_pt;
        vel_pts[2] << 0.0, 0.0, 0.0;
        double time_pts[] = {0, t_ss/2, t_ss};
        vector<Vector3d> swing_foot = TrajectoryPlanner::cubicPoly(way_pts, vel_pts, time_pts, dt_);

        for(int j=InitSSt_[i]; j<=EndSSt_[i]; j++){
            if(i%2 == 0){
                q1_[j] = swing_foot[j - InitSSt_[i]];
                q2_[j] = rF_[i+1];
            }else{
                q1_[j] = rF_[i+1];
                q2_[j] = swing_foot[j - InitSSt_[i]];
            }
        }
    }
    this->write2File(q1_ ,"q1");
    this->write2File(q2_ ,"q2");
}

void AnklePlanner::write2File(vector<Vector3d> input ,string file_name="data"){
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
int main(){
    vector<Vector3d> v1(9);
    v1[0] << 0.0, 0.12, 0.0;
    v1[1] << 0.0, -0.12, 0.0;
    v1[2] << 0.5, 0.12, 0.0;
    v1[3] << 1.0, -0.12, 0.0;
    v1[4] << 1.5, 0.12, 0.0;
    v1[5] << 2.0, -0.12, 0.0;
    v1[6] << 2.5, 0.12, 0.0;
    v1[7] << 3.0, -0.12, 0.0;
    v1[8] << 3.0, 0.12, 0.0;
    AnklePlanner d2(v1, 0.01, 0.8, 0.3, 0.3, 0.1);
    d2.phaseDetect();
    d2.setDSTraj();
    d2.setSSTraj();
    return 0;
}