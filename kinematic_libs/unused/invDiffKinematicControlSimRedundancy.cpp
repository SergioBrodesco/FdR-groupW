#include "../include/invDiffKinematicControlSimRedundancy.h"
#include <iostream>
#include <math.h>
using namespace Eigen;

// Simulates using Inverse Differential Kineamtics and Control
// A:parameters
// xd: desired position pf the of end effector (function of time)
// TH0: initial position of the joints
// minT, maxT: minimum and maximum time
// K: positive definite matrix used for control
// Dt: delta time

// Simulates using Inverse Differential Kinematics and Control

VectorXd qdot0_jlimits (const VectorXd& config){
    double k0 = 1;

    VectorXd wq (6);
    wq << ( (1/(6.14 - (-6.14))) * ((config(0) - (0))/(6.14 - (-6.14))) ),
          ( (1/(0 - (-3.14))) * ((config(1) - (-3.14/2))/(0 - (-3.14))) ),
          ( (1/(3.14 - (-3.14))) * ((config(2) - (0))/(3.14 - (-3.14))) ),

          ( (1/(6.28 - (-6.28))) * ((config(3) - (0))/(6.28 - (-6.28))) ),
          ( (1/(6.28 - (-6.28))) * ((config(4) - (0))/(6.28 - (-6.28))) ),
          ( (1/(6.28 - (-6.28))) * ((config(5) - (0))/(6.28 - (-6.28))) );

    return (-k0 * 1/6 * wq);
}

VectorXd qdot0_singularities (const VectorXd& config){
    double k0 = 20;

    MatrixXd J = ur5Jac(config);

    VectorXd wq = VectorXd::Ones(6) * sqrt((J * J.transpose()).determinant());

    return (-k0 * wq);
}


MatrixXd invDiffKinematicControlSimRedundancy(const MatrixXd& xd, const VectorXd& TH0, const double minT, const double maxT, const double Dt){
    
    Matrix3d K = 0.1 * Matrix3d::Identity();

    int L = xd.rows();

    VectorXd qk = TH0;
    MatrixXd q(L, TH0.size()+1);
    q(0,0) = xd(0, 0); q(0,1) = qk(0); q(0,2) = qk(1); q(0,3) = qk(2); q(0,4) = qk(3); q(0,5) = qk(4); q(0,6) = qk(5);
    
    VectorXd qdot0_i (6);

    for (int i = 1; i < L; i++)
    {
        std::tuple<Vector3d, Matrix3d> qk_conf = ur5Direct(qk);
        Vector3d xe = std::get<0>(qk_conf);

        VectorXd qdot_i = qdot0_singularities(qk);

        // Calcs
        Vector3d vd = (xd.row(i).segment(1, 3) - xd.row(i-1).segment(1, 3)) / Dt;
        VectorXd dotqk = invDiffKinematicControlRedundancy(qk, xe, xd.row(i).segment(1, 3), vd, qdot0_i, K);
        
        VectorXd qk1 = qk + dotqk * Dt;
        q(i,0) = xd(i,0); q(i,1) = qk1(0); q(i,2) = qk1(1); q(i,3) = qk1(2); q(i,4) = qk1(3); q(i,5) = qk1(4); q(i,6) = qk1(5);
        qk = qk1;
    }
    
    return q;
}