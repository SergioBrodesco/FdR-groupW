/**
 * @file motion_processor.h
 * @brief Header of the motion_processor.cpp file.
 * 
 * Motion planner is the ROS node we use to move the robot from a starting position to a desired one (received from task_planner.cpp)
 */
#ifndef MOTION_PROCESSOR_H
#define MOTION_PROCESSOR_H

#include "trajectory_planner.h"
#include "pos_manager.h"

#include <ros/callback_queue.h>

#include "lab_group_w/RobotInstructions.h"
#include "lab_group_w/ImageData.h"

// Receive Joint state
/**
 * @brief Callback function used for reading current jointStates from the /ur5/joint_states topic
 * 
 * @param jointState_msg_sim message obtained from the topic
 */
void recive_jstate(const sensor_msgs::JointState & jointState_msg_sim);
/// @brief vector containing the joint names, used to read the joint values in the recive_jstate() callback function
std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
/// @brief subscriber to the /ur5/joint_states topic
ros::Subscriber rec_jstate;

// Variables
/// @brief vector where we save the joint values obtained in recive_jstate()
std::vector<double> q (6, 0.0);

/// @brief diameter used for the gripper
double diameter;  //used for pos_manager
/// @brief Frequency used to coordinate with other nodes
const double loop_frequency = 1000.0;
/// @brief variable used for synchronization (will be used in p2pMotionPlan.cpp)
const double dt = 1/loop_frequency;

/**
 * @brief Set the positions in the "RobotInstructions" message using the information we received from the task planner node (task_planner.cpp)
 * 
 * @param p0 X
 * @param p1 Y
 * @param p2 Z
 */
void set_positions (double p0, double p1, double p2);
/**
 * @brief Set the rotations in the "RobotInstructions" message using the information we received from the task planner node (task_planner.cpp)
 * 
 * @param r0_0 
 * @param r0_1 
 * @param r0_2 
 * @param r1_0 
 * @param r1_1 
 * @param r1_2 
 * @param r2_0 
 * @param r2_1 
 * @param r2_2 
 */
void set_rotations (double r0_0, double r0_1, double r0_2, double r1_0, double r1_1, double r1_2, double r2_0, double r2_1, double r2_2);

/**
 * @brief Callback function used for reading the next position information received on the /task_planner/robot_instructions custom topic
 * 
 * @param msg RobotInstructions message received from the task planner node (task_planner.cpp) 
 */
void callback_instructions_sub(const lab_group_w::RobotInstructions & msg);
/**
 * @brief Function used to obtain the trajectory we want to follow, based on the information received from the task planner node (task_planner.cpp) 
 * 
 * @return Eigen::MatrixXd matrix containing the trajectory we'll send to the robot
 */
Eigen::MatrixXd get_trajectory();

// Set Subscribers
/// @brief subscriber to the /task_planner/robot_instructions topic
ros::Subscriber sub_instructions;
/// @brief message that the we receive from the task planner node (task_planner.cpp)
lab_group_w::RobotInstructions instructions;

/// @brief CallbackQueue used to coordinate when reading RobotInstructions from the /task_planner/robot_instructions topic
ros::CallbackQueue instructions_CallbackQueue;
/// @brief CallbackQueue used to coordinate when reading jstates from the /ur5/joint_states topic
ros::CallbackQueue jstates_CallbackQueue;

//set variables for service call "move_gripper" on real robot
/// @brief Set service client for move_gripper service
ros::ServiceClient gripper_client;
/// @brief message used for the move_gripper service (containing the diameter for the gripper)
ros_impedance_controller::generic_float srv;

#endif