/**
 * @file task_planner.h
 * @brief Header with the declaration of the functions implemented in task_planner.cpp.
 * 
 * task_planner is the ROS node we use to decide destinations for the robot arm
 */
#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "utils.h"
#include "lab_group_w/RobotInstructions.h"
#include "lab_group_w/ImageData.h"
#include "ros/ros.h"

#include <ros/callback_queue.h>

/// @brief message that the task planner sends to the motion node (motion_planner.cpp)
lab_group_w::RobotInstructions instructions;

/**
 * @brief Set the positions in the "RobotInstructions" message we'll send to the motion node
 * 
 * @param p0 X
 * @param p1 Y
 * @param p2 Z
 */
void set_positions (double p0, double p1, double p2);

/**
 * @brief Set the rotations in the "RobotInstructions" message we'll send to the motion node (saved as a Rotation matrix)
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
 * @brief Set the gripper diameter in the "RobotInstructions" message that we send to the motion node
 * 
 * @param class_type class type of the detected object
 * @param soft_gripper variable used to differentiate between soft and rigid gripper
 */
void set_diameter(int class_type, bool soft_gripper);

/**
 * @brief Function that decides where to put the blocks (calling set_positions()) depending on its class (saved inside instructions.class_type)
 * received from the image detection node
 * 
 */
void choose_destination ();

/// @brief publisher for the **/task_planner/robot_instructions** custom topic (RobotInstructions message)
ros::Publisher pub_instructions;

/// @brief subscriber to the **/imageProcessor/processed_data** custom topic (ImageData message)
ros::Subscriber sub_image_data;

/**
 * @brief Callback function used to store the informations of the detected object obtained from the /imageProcessor/processed_data custom topic
 * 
 * @param msg ImageData message received from the vision node
 */
void callback_sub(const lab_group_w::ImageData & msg);

/// @brief Frequency used to coordinate with other nodes
double  loop_frequency = 1000.;

/// @brief CallbackQueue for the **/imageProcessor/processed_data** custom topic, used to wait for messages
ros::CallbackQueue imageData_CallbackQueue;   // Set nodeHandler for imageData

#endif