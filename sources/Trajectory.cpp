#include "Trajectory.h"

vector<Vector3d> TrajectoryPlanner::cubicCoefs(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf){
    vector<Vector3d> coef(4);
    coef[0] = theta_ini;
    coef[1] = theta_dot_ini;
    coef[2] = (3 / pow(tf,2))*(theta_f - theta_ini) - (1/tf)*(2*theta_dot_ini + theta_dot_f);
    coef[3] = -(2 / pow(tf,3))*(theta_f - theta_ini) + (1 / pow(tf,2))*(theta_dot_ini + theta_dot_f);
    return coef;  
}

 vector<Vector3d> TrajectoryPlanner::cubicPoly(vector<Vector3d>& way_pts,vector<Vector3d>& vel_pts, double* time_pts, double dt){
     int point_count = way_pts.size();
     int length = (time_pts[point_count - 1] - time_pts[0])*1/dt;
     vector<Vector3d> q(length);
     const int coef_dim = 4;
     for(int i=0; i<length; i++){
         for(int j=0; j<point_count - 1; j++){
             if(i>=(time_pts[j]*1/dt) && i<(time_pts[j+1])){
                 double final_time = time_pts[j+1] - time_pts[j];
                 vector<Vector3d> coef = this->cubicCoefs(way_pts[j], way_pts[j+1], vel_pts[j], vel_pts[j+1], final_time);
                 q[i] = coef[0] + coef[1]*i*dt + coef[2]*pow(i*dt,2) + coef[3]*pow(i*dt,3);
                 //qd[i] = coef[1] + 2*coef[2]*i*dt + 3*coef[3]*pow(i*dt,2);
                 //qdd[i] = 2*coef[2] + 6*coef[3]*i*dt;
             }
         }
     }
     return q;
 }