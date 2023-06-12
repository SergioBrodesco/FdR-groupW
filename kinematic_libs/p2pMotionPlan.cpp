/**
 * @file p2pMotionPlan.cpp
 * @brief Implementation of the functions we use to compute the trajectories.
 * 
 * 
 */
#include "../include/p2pMotionPlan.h"


// xEs, phiEs: starting configuration for the end effector (joint-configuration + orientation)
// xEf, phiEf: final configuration for the end effector (joint-configuration + orientation)
// T: duration of the motion paln
// dt: sampling time

//
// Create trajectory in the shape of a parabolic curve of degree 3
//
std::tuple<MatrixXd, MatrixXd, MatrixXd> p2pMotionPlan(const VectorXd& xEs, const VectorXd& xEf, const double minT, const double maxT, const double dt, const int total_steps)
{
    VectorXd qEs = xEs;
    VectorXd qEf = xEf;
    
    // Calculate coefficients for each joint using a polynomial fit
    MatrixXd A(qEs.size(), 4);
    for (int i = 0; i < qEs.size(); i++) {
        MatrixXd M(4, 4);
        M << 1, minT, minT*minT, minT*minT*minT,
             0, 1, 2*minT, 3*minT*minT,

             1, maxT, maxT*maxT, maxT*maxT*maxT,
             0, 1, 2*maxT, 3*maxT*maxT;
        VectorXd b(4);
        b << qEs(i), 0, qEf(i), 0;

        VectorXd a = M.inverse() * b;
        A.row(i) = a.transpose();
    }


    MatrixXd Th (total_steps, 7);
    MatrixXd xE (total_steps, 4);
    MatrixXd phiE (total_steps, 4);

    // Generate the trajectory
    double t = minT;
    for (int k = 0; k < total_steps; k++) {
        VectorXd th(qEs.size());

        // Calculate joint positions at the current time using the coefficients
        for (int i = 0; i < qEs.size(); i++) {
            double q = A(i, 0) + A(i, 1)*t + A(i, 2)*t*t + A(i, 3)*t*t*t;
            th(i) = q;
        }

        // Calculate forward kinematics for the current joint configuration
        // (to obtain wrist position and rotation)
        Th.row(k) << t, fix_joint_config(th).transpose();

        auto DK_result = ur5Direct(th);
        VectorXd mx = std::get<0>(DK_result);
        MatrixXd mR = std::get<1>(DK_result);
        
        xE.row(k) << t, mx.transpose();
        phiE.row(k) << t, rotm2eulFDR(mR).transpose();
        
        t += dt;
    }
    return std::make_tuple(Th, xE, phiE);
}

double limitJointAngle(double angle, double minAngle, double maxAngle) {
    return std::max(minAngle, std::min(maxAngle, angle));
}

VectorXd fix_joint_config (const VectorXd& conf){
    VectorXd correction (6);

    correction << limitJointAngle(conf(0), -6.14, 6.14),
                  limitJointAngle(conf(1), -3.14, 0),
                  limitJointAngle(conf(2), -3.14, 3.14),
                  limitJointAngle(conf(3), -6.28, 6.28),
                  limitJointAngle(conf(4), -6.28, 6.28),
                  limitJointAngle(conf(5), -6.28, 6.28);

    return correction;
}


std::tuple<MatrixXd, MatrixXd, MatrixXd> p2via2pMotionPlan(const std::vector<VectorXd>& conf, const std::vector<double>& times, const double dt, const int total_steps){

    double minT = times.front();
    double maxT = times.back();

    MatrixXd velocity_profiles (conf.size(), 6);
    velocity_profiles.row(0) << 0, 0, 0, 0, 0, 0;
    velocity_profiles.row(1) << (fabs(conf[1](0) - conf[0](0))/times[1]), (fabs(conf[1](1) - conf[0](1))/times[1]), (fabs(conf[1](2) - conf[0](2))/times[1]), (fabs(conf[1](3) - conf[0](3))/times[1]), (fabs(conf[1](4) - conf[0](4))/times[1]), (fabs(conf[1](5) - conf[0](5))/times[1]);
    velocity_profiles.row(2) << 0, 0, 0, 0, 0, 0;

    velocity_profiles.row(1) = 0 * velocity_profiles.row(1);

    int num_cols = 7;

    MatrixXd Th (total_steps, num_cols);
    MatrixXd xE (total_steps, 4);
    MatrixXd phiE (total_steps, 4);

    int last_iteration = 0;
    // Calculate coefficients for each joint using a polynomial fit
    for (int k = 0; k < conf.size()-1; k++){
        MatrixXd A(6, 4);
        MatrixXd M(4, 4);

        double tA = times[k];
        double tB = times[k+1];

        for (int i = 0; i < 6; i++){
            M << 1, tA,   tA*tA,  tA*tA*tA,
                 0, 1,  2*tA,   3*tA*tA,    
                
                 1, tB,   tB*tB,  tB*tB*tB,
                 0, 1,  2*tB,   3*tB*tB;
                
            VectorXd b(4);
            b << conf[k](i),  velocity_profiles(k, i),
                 conf[k+1](i),  velocity_profiles(k+1, i);
                
            VectorXd a = M.inverse() * b;
            A.row(i) = a.transpose();
        }

        // Generate the trajectory
        double t = tA;
        int j = 0;
        while ((j+last_iteration)*dt < tB) {
            VectorXd th(6);

            // Calculate joint positions at the current time using the coefficients
            for (int i = 0; i < 6; i++) {
                double q = A(i, 0) + A(i, 1)*t + A(i, 2)*t*t + A(i, 3)*t*t*t;
                th(i) = q;
            }
            // Calculate forward kinematics for the current joint configuration
            // (to obtain wrist position and rotation)
            Th.row(j + last_iteration) << t, fix_joint_config(th).transpose();

            auto DK_result = ur5Direct(th);
            VectorXd mx = std::get<0>(DK_result);
            MatrixXd mR = std::get<1>(DK_result);
            
            xE.row(j + last_iteration) << t, mx.transpose();
            phiE.row(j + last_iteration) << t, rotm2eulFDR(mR).transpose();

            t += dt;
            j++;
        }

        last_iteration += j;
    }

    return std::make_tuple(Th, xE, phiE);
}