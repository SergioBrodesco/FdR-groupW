/**
 * @file invDirKinematics.cpp
 * @brief Implementation of the functions we use to compute the Direct and Inverse kinematics.
 * 
 * 
 */
#include "../include/invDirKinematics.h"

using namespace Eigen;

VectorXd A(6), D(6);

Matrix4d T10(double th1) {
    Matrix4d T;
    T << cos(th1), -sin(th1), 0, 0,
         sin(th1), cos(th1), 0, 0,
         0,        0,        1, 0.1625,
         0,        0,        0, 1;
    return T;
}
Matrix4d T21(double th2) {
    Matrix4d T;
    T << cos(th2), -sin(th2), 0, 0,
         0,        0,       -1, 0,
         sin(th2), cos(th2), 0, 0,
         0,        0,        0, 1;
    return T;
}
Matrix4d T32(double th3) {
    Matrix4d T;
    T << cos(th3), -sin(th3), 0, -0.425,
         sin(th3), cos(th3), 0, 0,
         0,        0,        1, 0,
         0,        0,        0, 1;
    return T;
}
Matrix4d T43(double th4) {
    Matrix4d T;
    T << cos(th4), -sin(th4), 0, -0.3922,
         sin(th4), cos(th4), 0, 0,
         0,        0,        1, 0.1333,
         0,        0,        0, 1;
    return T;
}
Matrix4d T54(double th5) {
    Matrix4d T;
    T << cos(th5), -sin(th5), 0, 0,
         0,        0,       -1, -0.0997,
         sin(th5), cos(th5), 0, 0,
         0,        0,        0, 1;
    return T;
}
Matrix4d T65(double th6) {
    Matrix4d T;
    T << cos(th6), -sin(th6), 0, 0,
         0,        0,        1, 0.0996,
        -sin(th6), -cos(th6), 0, 0,
         0,        0,        0, 1;
    return T;
}

std::tuple<Vector3d, Matrix3d> ur5Direct(VectorXd Th) {
  //Vector3d contiene posizione, Matrix3d contiene rotazione

  A(0) = 0, A(1) = -0.425, A(2) = -0.3922, A(3) = 0, A(4) = 0, A(5) = 0;
  D(0) = 0.1625, D(1) = 0, D(2) = 0, D(3) = 0.1333, D(4) = 0.0997, D(5) = 0.0996;
      
  Matrix4d T10m = T10(Th[0]);
  Matrix4d T21m = T21(Th[1]);
  Matrix4d T32m = T32(Th[2]);
  Matrix4d T43m = T43(Th[3]);
  Matrix4d T54m = T54(Th[4]);
  Matrix4d T65m = T65(Th[5]);
  
  Matrix4d T06 = T10m*T21m*T32m*T43m*T54m*T65m;

  Vector3d pe;  //end effector position
  pe << T06(0,3), T06(1,3), T06(2,3);

  MatrixXd Re(3,3); //end effector rotation
  Re << T06(0,0), T06(0,1), T06(0,2),
        T06(1,0), T06(1,1), T06(1,2),
        T06(2,0), T06(2,1), T06(2,2);

  return std::make_tuple(pe, Re);
}

MatrixXd ur5Inverse(Vector3d p60, Matrix3d R60) {

  A(0) = 0, A(1) = -0.425, A(2) = -0.3922, A(3) = 0, A(4) = 0, A(5) = 0;
  D(0) = 0.1625, D(1) = 0, D(2) = 0, D(3) = 0.1333, D(4) = 0.0997, D(5) = 0.0996;

  // Define Translation matrix
  Matrix4d T60;
  T60.topLeftCorner<3, 3>() = R60;
  T60.topRightCorner<3, 1>() = p60;
  T60.row(3) << 0, 0, 0, 1;

  // Finding th1
  VectorXd p50(4);
  p50 = T60 * Vector4d(0, 0, -D[5], 1);
  double hypot_p50 = hypot(p50[1], p50[0]);
  double th1_1 = std::real(std::atan2(p50[1], p50[0]) + std::acos(D[3]/hypot_p50) + M_PI/2.0);
  double th1_2 = std::real(std::atan2(p50[1], p50[0]) - std::acos(D[3]/hypot_p50) + M_PI/2.0);

  // Finding th5
  double th5_1 = +std::real(std::acos((p60[0]*std::sin(th1_1) - p60[1]*std::cos(th1_1) - D[3]) / D[5]));
  double th5_2 = -std::real(std::acos((p60[0]*std::sin(th1_1) - p60[1]*std::cos(th1_1) - D[3]) / D[5]));
  double th5_3 = +std::real(std::acos((p60[0]*std::sin(th1_2) - p60[1]*std::cos(th1_2) - D[3]) / D[5]));
  double th5_4 = -std::real(std::acos((p60[0]*std::sin(th1_2) - p60[1]*std::cos(th1_2) - D[3]) / D[5]));

  //related to th11 and th51
  Matrix4d T06_inv = T60.inverse();
  Vector3d Xhat = T06_inv.block<3,1>(0,0);
  Vector3d Yhat = T06_inv.block<3,1>(0,1);

  double th6_1 = std::real(atan2((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1))/sin(th5_1), (Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1))/sin(th5_1)));//related to th11 and th52
  double th6_2 = std::real(atan2((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1))/sin(th5_2), (Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1))/sin(th5_2)));//related to th12 and th53
  double th6_3 = std::real(atan2((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2))/sin(th5_3), (Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2))/sin(th5_3)));//related to th12 and th54
  double th6_4 = std::real(atan2((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2))/sin(th5_4), (Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2))/sin(th5_4)));

  Matrix4d T41m;

  T41m = T10(th1_1).inverse()*T60*T65(th6_1).inverse()*T54(th5_1).inverse();
  Vector3d p41_1 = T41m.block<3,1>(0,3);
  double p41xz_1 = hypot(p41_1(0), p41_1(2));

  T41m = T10(th1_1).inverse()*T60*T65(th6_2).inverse()*T54(th5_2).inverse();
  Vector3d p41_2 = T41m.block<3,1>(0,3);
  double p41xz_2 = hypot(p41_2(0), p41_2(2));

  T41m = T10(th1_2).inverse()*T60*T65(th6_3).inverse()*T54(th5_3).inverse();
  Vector3d p41_3 = T41m.block<3,1>(0,3);
  double p41xz_3 = hypot(p41_3(0), p41_3(2));

  T41m = T10(th1_2).inverse()*T60*T65(th6_4).inverse()*T54(th5_4).inverse();
  Vector3d p41_4 = T41m.block<3,1>(0,3);
  double p41xz_4 = hypot(p41_4(0), p41_4(2));

  //Computation of the 8 possible values for th3

  long double th3_1 = std::real(std::acos((std::pow(p41xz_1,2)-std::pow(A[1],2)-std::pow(A[2],2))/(2*A[1]*A[2])));
  long double th3_2 = std::real(std::acos((std::pow(p41xz_2,2)-std::pow(A[1],2)-std::pow(A[2],2))/(2*A[1]*A[2])));
  long double th3_3 = std::real(std::acos((std::pow(p41xz_3,2)-std::pow(A[1],2)-std::pow(A[2],2))/(2*A[1]*A[2])));
  long double th3_4 = std::real(std::acos((std::pow(p41xz_4,2)-std::pow(A[1],2)-std::pow(A[2],2))/(2*A[1]*A[2])));

  double th3_5 = -th3_1;
  double th3_6 = -th3_2;
  double th3_7 = -th3_3;
  double th3_8 = -th3_4;

  //Computation of eight possible values for th2
  double th2_1 = std::real(std::atan2(-p41_1(2), -p41_1(0))-std::asin((-A(2)*std::sin(th3_1))/p41xz_1));
  double th2_2 = std::real(std::atan2(-p41_2(2), -p41_2(0))-std::asin((-A(2)*std::sin(th3_2))/p41xz_2));
  double th2_3 = std::real(std::atan2(-p41_3(2), -p41_3(0))-std::asin((-A(2)*std::sin(th3_3))/p41xz_3));
  double th2_4 = std::real(std::atan2(-p41_4(2), -p41_4(0))-std::asin((-A(2)*std::sin(th3_4))/p41xz_4));

  double th2_5 = std::real(std::atan2(-p41_1(2), -p41_1(0))-std::asin((A(2)*std::sin(th3_1))/p41xz_1));
  double th2_6 = std::real(std::atan2(-p41_2(2), -p41_2(0))-std::asin((A(2)*std::sin(th3_2))/p41xz_2));
  double th2_7 = std::real(std::atan2(-p41_3(2), -p41_3(0))-std::asin((A(2)*std::sin(th3_3))/p41xz_3));
  double th2_8 = std::real(std::atan2(-p41_4(2), -p41_4(0))-std::asin((A(2)*std::sin(th3_4))/p41xz_4));

  MatrixXd T43m;
  VectorXd Xhat43;
  double th4_1, th4_2, th4_3, th4_4, th4_5, th4_6, th4_7, th4_8;

  T43m = T32(th3_1).inverse() * T21(th2_1).inverse() * T10(th1_1).inverse() * T60 * T65(th6_1).inverse() * T54(th5_1).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_1 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_2).inverse() * T21(th2_2).inverse() * T10(th1_1).inverse() * T60 * T65(th6_2).inverse() * T54(th5_2).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_2 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_3).inverse() * T21(th2_3).inverse() * T10(th1_2).inverse() * T60 * T65(th6_3).inverse() * T54(th5_3).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_3 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_4).inverse() * T21(th2_4).inverse() * T10(th1_2).inverse() * T60 * T65(th6_4).inverse() * T54(th5_4).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_4 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_5).inverse() * T21(th2_5).inverse() * T10(th1_1).inverse() * T60 * T65(th6_1).inverse() * T54(th5_1).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_5 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_6).inverse() * T21(th2_6).inverse() * T10(th1_1).inverse() * T60 * T65(th6_2).inverse() * T54(th5_2).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_6 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_7).inverse() * T21(th2_7).inverse() * T10(th1_2).inverse() * T60 * T65(th6_3).inverse() * T54(th5_3).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_7 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  T43m = T32(th3_8).inverse() * T21(th2_8).inverse() * T10(th1_2).inverse() * T60 * T65(th6_4).inverse() * T54(th5_4).inverse();
  Xhat43 = T43m.block<3,1>(0,0);
  th4_8 = std::real(std::atan2(Xhat43(1), Xhat43(0)));

  MatrixXd Th(8, 6);

  Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

  return Th;
}


/*
int main(){

  VectorXd q_0 (6);
  q_0 << -2.14503, -1.58197, -1.98971, -1.14071, -1.5708, 0.574238;
  // Compute current pose of the wrist

  std::tuple<Vector3d, Matrix3d> dp1 = ur5Direct(q_0);
  Vector3d pos_0 = std::get<0>(dp1);
  Matrix3d rot_0 = std::get<1>(dp1);
  
  std::cout << "POS_0 :" << std::endl;
  std::cout << pos_0.transpose() << std::endl;
  std::cout << "ROT_0 :" << std::endl;
  std::cout << rot_0 << std::endl << std::endl;

  VectorXd q_f (6);
  
  q_f << -2.02318, -1.43979, -1.66395, 3.37113, -0.987962, -2.41238;
  // Compute current pose of the wrost

  std::tuple<Vector3d, Matrix3d> dp2 = ur5Direct(q_f);
  Vector3d pos_f = std::get<0>(dp2);
  Matrix3d rot_f = std::get<1>(dp2);

  std::cout << "POS_F :" << std::endl;
  std::cout << pos_f.transpose() << std::endl;
  std::cout << "ROT_F :" << std::endl;
  std::cout << rot_f << std::endl << std::endl;

  return 0;
}
*/