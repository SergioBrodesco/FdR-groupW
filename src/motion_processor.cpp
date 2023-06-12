/**jointState_msgte_msg
 * @file motion_processor.cpp
 * @brief Implementation of the ROS node motion_processor, that we use to compute and send trajectories to the robot.
 * 
 * 
 */
#include "../include/motion_processor.h"
#include <unistd.h>

void recive_jstate(const sensor_msgs::JointState & jointState_msg){

  for(int i=0; i<jointState_msg.name.size(); i++) {
    for(int k=0; k<q.size(); k++) {
      if(jointState_msg.name[i] == std::string(joint_names[k])) {
        q[k] = jointState_msg.position[i];
        break;
      }
    }
  }
}
void callback_instructions_sub(const lab_group_w::RobotInstructions & msg) {
  std::cout << std::endl << "Received task" << std::endl;

  instructions.class_type = msg.class_type;

  instructions.move_gripper = msg.move_gripper;

  instructions.gripper_diameter = msg.gripper_diameter;

  set_positions(msg.position[0], msg.position[1], msg.position[2]);

  set_rotations(msg.rotation[0],msg.rotation[1],msg.rotation[2],
                msg.rotation[3],msg.rotation[4],msg.rotation[5],
                msg.rotation[6],msg.rotation[7],msg.rotation[8]
              );
  
}

void set_positions (double p0, double p1, double p2){
  instructions.position[0] = p0;
  instructions.position[1] = p1;
  instructions.position[2] = p2;
}
void set_rotations (double r0_0, double r0_1, double r0_2, double r1_0, double r1_1, double r1_2, double r2_0, double r2_1, double r2_2){
  instructions.rotation[0] = r0_0; instructions.rotation[1] = r0_1; instructions.rotation[2] = r0_2;
  instructions.rotation[3] = r1_0; instructions.rotation[4] = r1_1; instructions.rotation[5] = r1_2;
  instructions.rotation[6] = r2_0; instructions.rotation[7] = r2_1; instructions.rotation[8] = r2_2;
}


MatrixXd get_trajectory(){
  
  Vector3d object_pos_world;
  object_pos_world << instructions.position[0], instructions.position[1], instructions.position[2];

  Matrix3d object_rot_world;
  object_rot_world << instructions.rotation[0], instructions.rotation[1], instructions.rotation[2],
                      instructions.rotation[3], instructions.rotation[4], instructions.rotation[5],
                      instructions.rotation[6], instructions.rotation[7], instructions.rotation[8];

  // Configurazione attuale del robot
  VectorXd actual_joint_config (6);
  actual_joint_config << q[0], q[1], q[2], q[3], q[4], q[5];

  // Set gripper rotation
  Matrix3d gripper_rot;
  gripper_rot = object_rot_world; //Matrix3d::Identity();

  // Get object position and gripper rotation in Robot coordinate system
  Vector3d object_pos_robot = W_t_R_transform(object_pos_world);

  // Get possible configurations
  MatrixXd configurations (8,6);
  configurations = ur5Inverse(object_pos_robot,  gripper_rot);

  return get_best_config(configurations, actual_joint_config, object_pos_robot, dt);;
}

int main(int argc, char **argv)
{
  // Init node
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh_instructions;
  ros::NodeHandle nh_jstates;

  // Notice if it's real robot or not
  bool real_robot;
  nh_instructions.getParam("/real_robot", real_robot);
  if(real_robot) {
    gripper_client = nh_instructions.serviceClient<ros_impedance_controller::generic_float>("/move_gripper");
  }

  // setup callback queues
  nh_instructions.setCallbackQueue(& instructions_CallbackQueue);
  nh_jstates.setCallbackQueue(& jstates_CallbackQueue);

  // Set subscribers for robot instruction
  sub_instructions = nh_instructions.subscribe("/task_planner/robot_instructions", 30, callback_instructions_sub);
  rec_jstate = nh_jstates.subscribe("/ur5/joint_states", 30, recive_jstate);

  // Create pos manager
  Pos_manager p_manager = Pos_manager(nh_instructions);
  diameter = 150;

  ros::Rate loop_rate(loop_frequency);
  while (ros::ok())
  {
    while(!instructions_CallbackQueue.isEmpty())
    {
      instructions_CallbackQueue.callOne();
      jstates_CallbackQueue.callAvailable();
      
      MatrixXd trajectory = get_trajectory();
      
      VectorXd configuration_i (6);
      for (int i = 0; i < trajectory.rows(); i++)
      {
        configuration_i << trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4), trajectory(i,5), trajectory(i,6);

        if (instructions.move_gripper && i == trajectory.rows() - 1) {
          std::cout << "moving gripper\n";
          diameter = instructions.gripper_diameter;
          if(real_robot) {
            srv.request.data = diameter;
            loop_rate.sleep();
            if(gripper_client.call(srv))
              ROS_INFO("Service call succeeded");
            else
              ROS_ERROR("Service call failed");
          }
        }

        p_manager.send_Reference(configuration_i, diameter);
        jstates_CallbackQueue.callAvailable();
        loop_rate.sleep();        
      }
    }

    jstates_CallbackQueue.callAvailable();
  }

  return 0;
}