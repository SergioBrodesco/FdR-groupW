#include "../include/invDiffKinematicControlSimComplete.h"

using namespace Eigen;

//direct, Jac, xd, phid, TH0, Kp, Kphi, minT, maxT, Dt)
MatrixXd invDiffKinematicControlSimComplete(const MatrixXd& xd, const MatrixXd& phid , const VectorXd& TH0, const double minT, const double maxT, const double Dt){
    
    Matrix3d Kp = 1 * Matrix3d::Identity();
    Matrix3d Kphi = 0.1 * Matrix3d::Identity();

    int L = xd.rows();
    
    VectorXd qk = TH0;
    MatrixXd q(L, TH0.size()+1);
    q(0,0) = xd(0,0); q(0,1) = qk(0); q(0,2) = qk(1); q(0,3) = qk(2); q(0,4) = qk(3); q(0,5) = qk(4); q(0,6) = qk(5);
    
    for (int i = 1; i < L; i++)
    {
        std::tuple<Vector3d, Matrix3d> qk_conf = ur5Direct(qk);
        Vector3d xe = std::get<0>(qk_conf);
        Matrix3d Re = std::get<1>(qk_conf);
        Vector3d phie = rotm2eulFDR(Re);

        // Calcs
        Vector3d vd = (xd.row(i).segment(1, 3) - xd.row(i-1).segment(1, 3)) / Dt;
        Vector3d phiddot = (phid.row(i).segment(1, 3) - phid.row(i-1).segment(1, 3)) / Dt;

        // Call the funtion
        VectorXd dotqk = invDiffKinematicControlComplete(qk, xe, xd.row(i).segment(1, 3), vd, phie, phid.row(i).segment(1, 3), phiddot, Kp, Kphi);
        
        VectorXd qk1 = qk + dotqk * Dt;
        q(i,0) = xd(i,0); q(i,1) = qk1(0); q(i,2) = qk1(1); q(i,3) = qk1(2); q(i,4) = qk1(3); q(i,5) = qk1(4); q(i,6) = qk1(5);
        qk = qk1;
    }
    
    return q;
}