/**
 * @file pos_manager.h
 * @brief Header of the class Pos_manager, used to handle publishers.
 */
#ifndef POS_MANAGER_H
#define POS_MANAGER_H

#include "utils.h"
#include "ros/ros.h"
#include "ros_impedance_controller/generic_float.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>


/**
 * @brief pos_manager is a class that we use to publish joint angles for the real/simulated robot
 * 
 */
class Pos_manager {
public:
  /**
   * @brief Constructor for a new Pos_manager object, differentiate between real and simulated robot, and soft/rigid gripper
   * 
   * @param node ros node we want this object to have reference to
   */
  Pos_manager(ros::NodeHandle node);
  /**
   * @brief Function used to publish robot instructions, calls send_full_joint_state() or send_reduced_joint_state(),
   * depending on how much information we can publish
   * 
   * @param q_des joint angle we want to publish
   * @param diameter diameter for opening or closing the gripper
   * @param qd_des joint velocity we want to publish
   * @param tau_ffwd joint effort we want to publish
   */
  void send_Reference(Eigen::VectorXd q_des, double diameter=0, Eigen::VectorXd qd_des = Eigen::VectorXd(), Eigen::VectorXd tau_ffwd = Eigen::VectorXd());

private:
  /**
   * @brief Function that initializes a filter (used fon gripper fingers in simulation)
   * 
   * @param size size of the filter
   */
  void initFilter(const int & size);
  /**
   * @brief Function that applies the filter on a given input (gripper joint in simulation)
   * 
   * @param input vector we want to filter
   * @param rate frequency at which the filter is being applied
   * @param settling_time settling time of the filter
   * @return Eigen::VectorXd filtered output obtained from the input vector
   */
  Eigen::VectorXd secondOrderFilter(Eigen::VectorXd input, const double rate, const double settling_time);
  /**
   * @brief Function used to publish when using "torque" control mode in simulation
   * 
   * @param q_des joint angle we want to publish
   * @param diameter diameter for opening or closing the gripper
   * @param qd_des joint velocity we want to publish
   * @param tau_ffwd joint effort we want to publish
   */
  void send_full_joint_state(Eigen::VectorXd q_des, double diameter=0, Eigen::VectorXd qd_des = Eigen::VectorXd(), Eigen::VectorXd tau_ffwd = Eigen::VectorXd());
  /**
   * @brief Function used to only publish joint and gripper positions
   * 
   * @param q_des joint angle we want to publish
   * @param diameter diameter for opening or closing the gripper
   */
  void send_reduced_joint_state(Eigen::VectorXd q_des, double diameter=0);
  
  /// @brief node we use to instantiate the publishers
  ros::NodeHandle pos_manager_node;
  /// @brief boolean representing real or simulated robot (true = REAL ROBOT, false = SIMULATION)
  bool real_robot;
  /// @brief boolean to see if we want to utilize the gripper during simulation
  bool gripper_sim;
  /// @brief boolean differentiating between soft and rigid gripper (true = SOFT GRIPPER, false = RIGID GRIPPER)
  bool soft_gripper;
  /// @brief number of fingers decided from the soft_gripper variable
  int number_of_fingers;
  /// @brief number of joints of the ur5 robot exluding the gripper fingers (in our case its always 6)
  int number_of_joints;
  /// @brief string representing the control type ("position" or "torque", we only use position)
  std::string control_type;
  /// @brief message we publish in the send_full_joint_state() function
  sensor_msgs::JointState jointState_msg_sim;
  /// @brief message we publish in the send_reduced_joint_state() function
  std_msgs::Float64MultiArray jointState_msg_robot;
  /// @brief publisher for joint states (subscribes to a different topic, depending on real_robot value)
  ros::Publisher pub_des_jstate;
  /// @brief service client to make a service call to move_gripper() when using the real robot
  ros::ServiceClient gripper_client;
  /// @brief message we use for the rosservice call to move_gripper()
  ros_impedance_controller::generic_float srv;
  /// @brief first filter used on secondOrderFilter()
  Eigen::VectorXd filter_1;
  /// @brief second filter used on secondOrderFilter()
  Eigen::VectorXd filter_2;
};

#endif