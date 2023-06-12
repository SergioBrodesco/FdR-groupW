/**
 * @file trajectory_planner.h
 * @brief Header with the declaration of the functions implemented in trajectory_planner.cpp
 * 
 * 
 */
#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "invDiffKinematicControlSimCompleteAngleAxis.h"
#include "p2pMotionPlan.h"
#include "collision_handler.h"

/**
 * @brief Returns the trajectory that we use to reach a goal point
 * 
 * @param Th joint configurations obtained from inverse kinematic
 * @param q_0 initial joint configuration
 * @param goal_point point we want the end effector to reach
 * @param dt time interval that we use to decide how many points a trajectory should have
 * @return MatrixXd containing the trajectory we'll use to reach the desired point
 */
MatrixXd get_best_config(const MatrixXd& Th, const VectorXd& q_0, const Vector3d& goal_point, const double dt);  //find the best joint configuration after inv_kinematics

/**
 * @brief Function that checks if the joint configurations obtained from inverse kinematic are vaild
 * 
 * @param Th joint configurations obtained from inverse kinematic
 * @param goal_point point we want the end effector to reach
 * @return std::array<bool,8> array containing the validity of the configurations (true = VALID, false = NOT VALID)
 */
std::array<bool,8> exclude_invalid_confs (const MatrixXd& Th, const Vector3d& goal_point);

/**
 * @brief Function that sorts the valid configurations from the one that has the minimum difference in
 * overall joint angles with the starting configuration
 * 
 * @param Th joint configurations obtained from inverse kinematic
 * @param q_0 initial joint configuration
 * @param valid_config array containing the validity of the configurations (true = VALID, false = NOT VALID)
 * @return std::array<int, 8> sorted vector that contains the ordered indexes of the joint configurations inside Th
 */
std::array<int, 8> sort_confs (const MatrixXd& Th, const VectorXd& q_0, const std::array<bool, 8>& valid_config);

/**
 * @brief Function that decides the time a trajectory should take to complete
 * 
 * @param q_0 joint configuration at the start of the trajectory
 * @param q_f joint configuration at the end of the trajectory 
 * @param scaling_factor scaling factor that multiplies the maximum joint velocity in order to scale it down (we don't want the joint to move at maximum speed)
 * @param minimum_time minimum time that is added to the computed trajectory time to have even more control over the final computed time
 * @return double maxT used to compute the trajectory between the two passed joint configurations
 */
double find_optimal_maxT (const VectorXd& q_0 , const VectorXd& q_f, const double scaling_factor, const double minimum_time);

#endif