/**
 * @file invDiffKinematicControlSimCompleteAngleAxis.cpp
 * @brief Implementation of the functions we use to compute the Differential kinematic
 * 
 * 
 */
#include "../include/invDiffKinematicControlSimCompleteAngleAxis.h"

using namespace Eigen;

Vector3d computeOrientationErrorW(Matrix3d w_R_e, Matrix3d w_R_d){
    Vector3d errorW;

    //compute relative orientation 
    Matrix3d e_R_d = w_R_e.transpose() * w_R_d;

   //compute the delta_angle
   double cos_dtheta = (e_R_d(0,0) + e_R_d(1,1) + e_R_d(2,2) -1)/2;
   Vector3d tmp;
   tmp << e_R_d(2,1) -e_R_d(1,2),
          e_R_d(0,2) -e_R_d(2,0),
          e_R_d(1,0) -e_R_d(0,1);

   double sin_dtheta = tmp.norm() / 2;

   double dtheta= atan2(sin_dtheta, cos_dtheta);

   if (dtheta == 0) 
    errorW = Vector3d::Zero();
   else{
     Vector3d axis;
     axis << e_R_d(2,1) -e_R_d(1,2),
             e_R_d(0,2) -e_R_d(2,0),
             e_R_d(1,0) -e_R_d(0,1);
     axis = 1/(2*sin_dtheta)*axis;
             
     errorW = w_R_e * axis*dtheta;
   }

    return errorW;
}

Matrix3d get_optimal_Kphi (const VectorXd& start_cfg, const VectorXd& end_cfg, const float base_factor){
    auto res_s = ur5Direct(start_cfg);
    Matrix3d Re_start = std::get<1>(res_s);
    Vector3d rvec_start = rotm2eulFDR(Re_start);

    auto res_e = ur5Direct(end_cfg);
    Matrix3d Re_end = std::get<1>(res_e);
    Vector3d rvec_end = rotm2eulFDR(Re_end);

    Vector3d Ks;
    Ks << base_factor + fabs(rvec_end(0) - rvec_start(0)),
          base_factor + fabs(rvec_end(1) - rvec_start(1)),
          base_factor + fabs(rvec_end(2) - rvec_start(2));

    return Ks.asDiagonal();
}

VectorXd invDiffKinematicControlCompleteAngleAxis(const VectorXd& q, const Vector3d& xe, const Vector3d& xd, const Vector3d& vd, const Matrix3d& w_R_e, const Vector3d& phid, const Vector3d& phiddot, const Matrix3d& Kp, const Matrix3d& Kphi){
    Matrix3d w_R_d = eul2rotmFDR(phid);
    Vector3d error_o = computeOrientationErrorW(w_R_e, w_R_d);

    MatrixXd J = ur5Jac(q) + MatrixXd::Identity(6,6)*1e-06;

    VectorXd correction (6);
    correction << (vd + Kp*(xd-xe)), (Kphi*(error_o));

    VectorXd dotQ = J.inverse() * correction;

    return dotQ;
}

std::tuple<MatrixXd, MatrixXd, MatrixXd> invDiffKinematicControlSimCompleteAngleAxis(const MatrixXd& xd, const MatrixXd& phid , const VectorXd& TH0, const VectorXd& THf, const double minT, const double maxT, const double Dt){
    
    Matrix3d Kp = 5 * Matrix3d::Identity();
    Matrix3d optimal_kphi = get_optimal_Kphi(TH0, THf, 0.1);
    Matrix3d Kphi = 5 * optimal_kphi;

    int L = xd.rows();
    
    VectorXd qk = TH0;
    MatrixXd q(L, TH0.size()+1);
    MatrixXd xe_out(L , 4);
    MatrixXd phie_out(L, 4);

    q.row(0) << xd(0,0), qk.transpose();
    
    for (int i = 1; i < L; i++)
    {
        std::tuple<Vector3d, Matrix3d> qk_conf = ur5Direct(qk);
        Vector3d xe = std::get<0>(qk_conf);
        Matrix3d Re = std::get<1>(qk_conf);

        Vector3d vd = (xd.row(i).segment(1, 3) - xd.row(i-1).segment(1, 3)) / Dt;
        Vector3d phiddot = (phid.row(i).segment(1, 3) - phid.row(i-1).segment(1, 3)) / Dt;

        // Get optimal velocities
        VectorXd dotqk = invDiffKinematicControlCompleteAngleAxis(qk, xe, xd.row(i).segment(1, 3), vd, Re, phid.row(i).segment(1, 3), phiddot, Kp, Kphi);
        
        VectorXd qk1 = qk + dotqk * Dt;

        q.row(i) << xd(i,0), qk1.transpose();
        
        std::tuple<Vector3d, Matrix3d> qk1_conf = ur5Direct(qk1);
        xe = std::get<0>(qk1_conf);
        Re = std::get<1>(qk1_conf);
        xe_out.row(i) << xd(i,0), xe.transpose();
        phie_out.row(i) << xd(i,0), rotm2eulFDR(Re).transpose();

        qk = qk1;
    }
    
    return std::make_tuple(q, xe_out, phie_out);
}