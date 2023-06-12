/**
 * @file pos_manager.cpp
 * @brief Implementation of the class we use to comunicate with the real/simulated robot.
 * 
 * 
 */
#include "../include/pos_manager.h"

Pos_manager::Pos_manager(ros::NodeHandle node) : pos_manager_node(node, "pos_manager") {
  
  node.getParam("/real_robot", real_robot);
  node.getParam("/gripper_sim", gripper_sim);
  node.getParam("/soft_gripper", soft_gripper);
  node.getParam("/control_type", control_type);
  number_of_joints=6;

  if(gripper_sim) {   //3 finger gripper ha 3 joint in più
    if(soft_gripper) {
      number_of_fingers=2;

    } else {
      number_of_fingers=3;
    }
  } else {
    number_of_fingers=0;
  }

  if(real_robot) {
    gripper_client = node.serviceClient<ros_impedance_controller::generic_float>("/move_gripper");
    number_of_fingers=0;    //apertura/chiusura gripper deve essere gestita in modo diverso con il robot reale
  }

  std::cout << "NUMBER OF FINGERS : " << number_of_fingers << std::endl;

  initFilter(number_of_fingers);

  jointState_msg_sim.position.resize(number_of_joints + number_of_fingers);
  jointState_msg_sim.velocity.resize(number_of_joints + number_of_fingers);
  jointState_msg_sim.effort.resize(number_of_joints + number_of_fingers);
  jointState_msg_robot.data.resize(number_of_joints + number_of_fingers);

  if (control_type == "torque" && !real_robot) {    //noi non utilizziamo torque
    pub_des_jstate = pos_manager_node.advertise<sensor_msgs::JointState>("/command", 1);
  } else {
    pub_des_jstate = pos_manager_node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
  }
}

void Pos_manager::initFilter(const int & size)
{
  filter_1 = Eigen::VectorXd::Zero(size);
  filter_2 = Eigen::VectorXd::Zero(size);
}

Eigen::VectorXd Pos_manager::secondOrderFilter(Eigen::VectorXd input, const double rate, const double settling_time)
{
  double dt = 1 / rate;
  double gain =  dt / (0.1*settling_time + dt);
  filter_1 = (1 - gain) * filter_1 + gain * input;
  filter_2 = (1 - gain) * filter_2 + gain * filter_1;
  return filter_2;
}

void Pos_manager::send_full_joint_state(Eigen::VectorXd q_des, double diameter, Eigen::VectorXd qd_des, Eigen::VectorXd tau_ffwd) {
  for(int i=0; i<q_des.size(); i++) {
    jointState_msg_sim.position[i] = q_des[i];
    jointState_msg_sim.velocity[i] = qd_des[i];
    jointState_msg_sim.effort[i] = tau_ffwd[i];
  }
  pub_des_jstate.publish(jointState_msg_sim);
}

void Pos_manager::send_reduced_joint_state(Eigen::VectorXd q_des, double diameter) {
  for(int i=0; i<q_des.size(); i++) {
    jointState_msg_robot.data[i] = q_des[i];
  }

  if(!real_robot && gripper_sim) {
    
    double finger_pos;
    if(soft_gripper) {
      double D0 = 40;
      double L = 60;
      double delta = 0.5 * (diameter - D0);
      finger_pos = std::atan2(delta, L);
    } else {
      finger_pos = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    }
    
    Eigen::VectorXd tmp = Eigen::VectorXd::Constant(number_of_fingers, finger_pos);
    tmp = secondOrderFilter(tmp, 0.001, 2.5);

    for(int i=number_of_joints; i<(number_of_joints + number_of_fingers); i++) {
      jointState_msg_robot.data[i] = finger_pos;
    }
    
  } else if(real_robot && !gripper_sim) {
    //rosservice call a "/move_gripper"
    //viene effettuata in "custom_joint_publisher.cpp" per evitare che venga chiamata ad ogni pubblicazione
    /*srv.request.data = diameter;
    if(gripper_client.call(srv))
      ROS_INFO("Service call succeeded");
    else
      ROS_ERROR("Service call failed");*/
  }

  pub_des_jstate.publish(jointState_msg_robot);
}

void Pos_manager::send_Reference(Eigen::VectorXd q_des, double diameter, Eigen::VectorXd qd_des, Eigen::VectorXd tau_ffwd) {
  if(control_type == "torque" && !real_robot) {    //torque control non si può usare con real robot
    if(qd_des.size() == 0)
      qd_des = Eigen::VectorXd::Zero(6);
    if(tau_ffwd.size() == 0)
      tau_ffwd = Eigen::VectorXd::Zero(6);
    send_full_joint_state(q_des, diameter, qd_des, tau_ffwd);
  } else {
    send_reduced_joint_state(q_des, diameter);
  }
}