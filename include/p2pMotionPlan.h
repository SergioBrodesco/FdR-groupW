/**
 * @file p2pMotionPlan.h
 * @brief Header of the p2pMotionPlan.cpp file, used to compute trajectories
 * 
 */
#ifndef P2PMOTIONPLAN_H
#define P2PMOTIONPLAN_H

#include "invDirKinematics.h"

using namespace Eigen;

/**
 * @brief Function that computes trajectories using a cubic polynomial
 * 
 * @param xEs starting end effector position
 * @param xEf final end effector position
 * @param minT start time for the motion plan
 * @param maxT finish time for the motion plan
 * @param dt sampling time
 * @param total_steps number of configurations we'll sample
 * @return std::tuple<MatrixXd, MatrixXd, MatrixXd> computed trajectory (joint positions, end effector positions, end effector rotations)
 */
std::tuple<MatrixXd, MatrixXd, MatrixXd> p2pMotionPlan(const VectorXd& xEs, const VectorXd& xEf, const double minT, const double maxT, const double dt, const int total_steps);
//std::tuple<MatrixXd, MatrixXd, MatrixXd> p2via2pMotionPlan(const VectorXd& xEs, const VectorXd& xEvia, const VectorXd& xEf, const double minT, const double maxT, const double dt);

/**
 * @brief Function that computes trajectories using a cubic polynomial, adapted to recieve multiple points to pass trough (NOT USED)
 * 
 * @param conf points we want our trajectory to cover
 * @param times times at which we want to reach each point
 * @param dt sampling time
 * @param total_steps number of configurations we'll sample
 * @return std::tuple<MatrixXd, MatrixXd, MatrixXd> computed trajectory (joint positions, end effector positions, end effector rotations)
 */
std::tuple<MatrixXd, MatrixXd, MatrixXd> p2via2pMotionPlan(const std::vector<VectorXd>& conf, const std::vector<double>& times, const double dt, const int total_steps);
/**
 * @brief Function used to "correct" joint angles if they go beyond their limits, the actual modification is done by calling limitJointAngle()
 * 
 * @param conf joint angles we want to check
 * @return VectorXd corrected joint angles
 */
VectorXd fix_joint_config (const VectorXd& conf);
/**
 * @brief Function that checks if a recieved joint angle surpasses its limits and eventually modifies it.\n
 * This is called for every joint by fix_joint_config()
 * 
 * @param angle joint angle we want to check
 * @param minAngle minimum value for the recieved joint
 * @param maxAngle maximum value for the recieved joint
 * @return double corrected joint angle
 */
double limitJointAngle(double angle, double minAngle, double maxAngle);


#endif
