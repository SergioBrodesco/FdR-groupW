/**
 * @file ur5Jac.cpp
 * @brief implementation of the Jacobian for the ur5
 * 
 */
#include "../include/ur5Jac.h"

using namespace Eigen;

MatrixXd ur5Jac(VectorXd Th) {
  VectorXd A(6), D(6);
  A << 0, -0.425, -0.3922, 0, 0, 0;
  D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
  
  double A1 = A(0), A2 = A(1), A3 = A(2), A4 = A(3), A5 = A(4), A6 = A(5);
  double D1 = D(0), D2 = D(1), D3 = D(2), D4 = D(3), D5 = D(4), D6 = D(5);
  
  double th1 = Th(0), th2 = Th(1), th3 = Th(2), th4 = Th(3), th5 = Th(4), th6 = Th(5);
  
  VectorXd J1(6), J2(6), J3(6), J4(6), J5(6), J6(6);

  J1 << D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1),
        D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D3*sin(th1) + D4*sin(th1) + A3*cos(th2 + th3)*cos(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1),
        0,
        0,
        0,
        1;
        
  J2 << -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
        -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
        A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
        sin(th1),
        -cos(th1),
        0;

  J3 << cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
        sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
        A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
        sin(th1),
        -cos(th1),
        0;
    
  J4 << D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
        D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
        D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
        sin(th1),
        -cos(th1),
        0;
     
  J5 << -D5*sin(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th1)*cos(th5),
        D5*cos(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th5)*sin(th1),
        -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
        sin(th2 + th3 + th4)*cos(th1),
        sin(th2 + th3 + th4)*sin(th1),
        -cos(th2 + th3 + th4);
  J6 << 0,
        0,
        0,
        cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
        -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
        -sin(th2 + th3 + th4)*sin(th5);

  MatrixXd J (6,6);
  J << J1(0),J2(0),J3(0),J4(0),J5(0),J6(0),
       J1(1),J2(1),J3(1),J4(1),J5(1),J6(1),
       J1(2),J2(2),J3(2),J4(2),J5(2),J6(2),
       J1(3),J2(3),J3(3),J4(3),J5(3),J6(3),
       J1(4),J2(4),J3(4),J4(4),J5(4),J6(4),
       J1(5),J2(5),J3(5),J4(5),J5(5),J6(5);

  return J;
}