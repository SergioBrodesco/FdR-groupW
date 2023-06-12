#include "../include/invDiffKinematicControlComplete.h"

using namespace Eigen;


//phie, phid, phiddot, Kp, Kphi)
MatrixXd invDiffKinematicControlComplete(const VectorXd& q, const Vector3d& xe, const Vector3d& xd, const Vector3d& vd, const Vector3d& phie, const Vector3d& phid, const Vector3d& phiddot, const Matrix3d& Kp, const Matrix3d& Kphi){

    MatrixXd J = ur5Jac(q);
    double psi = phie(0);
    double theta = phie(1);
    double phi = phie(2);

    Matrix3d T;
    T <<  cos(theta)*cos(phi), -sin(phi), 0,
          cos(theta)*sin(phi), cos(phi),  0,
          -sin(theta),           0,           1;

    // Define Translation matrix
    MatrixXd Ta (6,6);
    Ta.topLeftCorner<3, 3>() = Matrix3d::Identity();
    Ta.topRightCorner<3, 3>() = Matrix3d::Zero();
    Ta.bottomLeftCorner<3, 3>() = Matrix3d::Zero();
    Ta.bottomRightCorner<3, 3>() = T;

    MatrixXd Ja = Ta.inverse() * J;

    VectorXd correction (6);
    correction << (vd + Kp*(xd-xe)), (phiddot+Kphi*(phid-phie));

    MatrixXd dotQ = (Ja + MatrixXd::Identity(6,6)*1e-06).inverse() * correction;

    return dotQ;
}
