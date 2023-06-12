/**
 * @file task_planner.cpp
 * @brief Implementation of the ROS node task_planner, that we use to send destinations to the motion node.
 * 
 * 
 */
#include "../include/task_planner.h"

void callback_sub(const lab_group_w::ImageData & msg)
{  
  std::cout << "Message received" << std::endl;

  instructions.class_type = msg.class_type;

  set_positions(msg.position[0], msg.position[1], msg.position[2]);

  set_rotations(msg.rotation[0], msg.rotation[1], msg.rotation[2],
                msg.rotation[3], msg.rotation[4], msg.rotation[5],
                msg.rotation[6], msg.rotation[7], msg.rotation[8]
              );
}

void choose_destination (){
  // y1 .302851
  // y2 .437044
  // y3 .570141
  // y4 .703511

  // x1 .662487
  // x2 .785764
  // x3 .8957

  switch (instructions.class_type){
    
    // X3
    case 0:  set_positions(0.8957, 0.302851, 0.866); break;  // X1-Y1-Z2
    case 1:  set_positions(0.8957, 0.437044, 0.866); break;  // X1-Y2-Z1
    case 2:  set_positions(0.8957, 0.570141, 0.866); break;  // X1-Y2-Z2
    case 3:  set_positions(0.8957, 0.703511, 0.866); break;  // X1-Y2-Z2-CHAMFER
    
    // X2
    case 4:  set_positions(0.785764, 0.302851, 0.866); break;  // X1-Y2-Z2-TWINFILLET
    case 5:  set_positions(0.785764, 0.437044, 0.866); break;  // X1-Y3-Z2
    case 6:  set_positions(0.785764, 0.570141, 0.866); break;  // X1-Y3-Z2-FILLET
    case 10: set_positions(0.785764, 0.703511, 0.866); break;  // X2-Y2-Z2-FILLET

    // X1
    case 7:  set_positions(0.662487, 0.437044, 0.866); break;  // X1-Y4-Z1
    case 8:  set_positions(0.662487, 0.570141, 0.866); break;  // X1-Y4-Z2
    case 9:  set_positions(0.662487, 0.703511, 0.866); break;  // X2-Y2-Z2
  }
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

void set_diameter(int class_type, bool soft_gripper){
  if(soft_gripper){
    switch (class_type){
      case 0:  instructions.gripper_diameter = 15; break; // X1-Y1-Z2
      case 1:  instructions.gripper_diameter = 15; break; // X1-Y2-Z1
      case 2:  instructions.gripper_diameter = 15; break; // X1-Y2-Z2
      case 3:  instructions.gripper_diameter = 15; break; // X1-Y2-Z2-CHAMFER
      case 4:  instructions.gripper_diameter = 15; break; // X1-Y2-Z2-TWINFILLET
      case 5:  instructions.gripper_diameter = 15; break; // X1-Y3-Z2
      case 6:  instructions.gripper_diameter = 15; break; // X1-Y3-Z2-FILLET
      case 7:  instructions.gripper_diameter = 15; break; // X1-Y4-Z1
      case 8:  instructions.gripper_diameter = 15; break; // X1-Y4-Z2
      case 9:  instructions.gripper_diameter = 35; break; // X2-Y2-Z2
      case 10: instructions.gripper_diameter = 35; break; // X2-Y2-Z2-FILLET
    }
  }
  else{
    switch (class_type){
      case 0:  instructions.gripper_diameter = 40; break; // X1-Y1-Z2
      case 1:  instructions.gripper_diameter = 40; break; // X1-Y2-Z1
      case 2:  instructions.gripper_diameter = 40; break; // X1-Y2-Z2
      case 3:  instructions.gripper_diameter = 40; break; // X1-Y2-Z2-CHAMFER
      case 4:  instructions.gripper_diameter = 40; break; // X1-Y2-Z2-TWINFILLET
      case 5:  instructions.gripper_diameter = 40; break; // X1-Y3-Z2
      case 6:  instructions.gripper_diameter = 40; break; // X1-Y3-Z2-FILLET
      case 7:  instructions.gripper_diameter = 40; break; // X1-Y4-Z1
      case 8:  instructions.gripper_diameter = 40; break; // X1-Y4-Z2
      case 9:  instructions.gripper_diameter = 40; break; // X2-Y2-Z2
      case 10: instructions.gripper_diameter = 40; break; // X2-Y2-Z2-FILLET
    }
  } 
}


int main(int argc, char **argv)
{
  // Init node
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle node;
  ros::NodeHandle nh_imageData ;

  // Set callback queue
  nh_imageData.setCallbackQueue(& imageData_CallbackQueue);

  // Get if soft gripper is set
  bool soft_gripper;
  node.getParam("/soft_gripper", soft_gripper);

  // Set publishers and subscribers
  pub_instructions = node.advertise<lab_group_w::RobotInstructions>("/task_planner/robot_instructions", 30);
  sub_image_data = nh_imageData.subscribe("/imageProcessor/processed_data", 11, callback_sub);

  ros::Rate loop_rate(loop_frequency);
  while (ros::ok())
  {
    // Process instructions only once imageData are available
    if(!imageData_CallbackQueue.isEmpty())
    {
      imageData_CallbackQueue.callOne();
      std::cout << "Processing robot instructions..." << std::endl << std::endl;

      // Define Robot action
      double safe_heigth = 0.35;

      double gripper_offset;
      if(soft_gripper)
        gripper_offset = 0.128;
      else
        gripper_offset = 0.165;
      
      set_diameter(instructions.class_type, soft_gripper);

      double opened_gripper = 150;
      double closed_gripper = instructions.gripper_diameter;

      // Save values apart
      double obj_height = instructions.position[2];
      double obj_rot [9] {instructions.rotation[0], instructions.rotation[1], instructions.rotation[2],
                          instructions.rotation[3], instructions.rotation[4], instructions.rotation[5],
                          instructions.rotation[6], instructions.rotation[7], instructions.rotation[8]};
      
      // 6 points Trajectory
      for (int i = 0; i < 6; i++){
        // Go over the object
        if (i == 0){
          instructions.position[2] = (obj_height + safe_heigth);
          instructions.move_gripper = false;
          instructions.gripper_diameter = opened_gripper;
          set_rotations(1, 0, 0, 0, 1, 0, 0, 0, 1);

          pub_instructions.publish(instructions);
        }
        // Approach and grab the object
        else if (i == 1){
          instructions.position[2] = (obj_height + gripper_offset);
          instructions.move_gripper = true;
          instructions.gripper_diameter = closed_gripper;
          set_rotations(obj_rot[0], obj_rot[1], obj_rot[2], obj_rot[3], obj_rot[4], obj_rot[5], obj_rot[6], obj_rot[7], obj_rot[8]);
          
          pub_instructions.publish(instructions);
        }
        // get up again, neutral rotation
        else if (i == 2){
          instructions.position[2] = (obj_height + safe_heigth);
          instructions.move_gripper = false;
          instructions.gripper_diameter = closed_gripper;
          set_rotations(1, 0, 0, 0, 1, 0, 0, 0, 1);

          pub_instructions.publish(instructions);
        }
        // Go over landing position
        else if (i == 3){
          choose_destination();
          instructions.position[2] = (obj_height + safe_heigth);
          instructions.move_gripper = false;
          instructions.gripper_diameter = closed_gripper;
          set_rotations(1, 0, 0, 0, 1, 0, 0, 0, 1);

          pub_instructions.publish(instructions);
        }
        // Approach landing position and drop the object
        else if (i == 4){
          instructions.position[2] = (obj_height + gripper_offset);
          instructions.move_gripper = true;
          instructions.gripper_diameter = opened_gripper;
          set_rotations(1, 0, 0, 0, 1, 0, 0, 0, 1);

          pub_instructions.publish(instructions);
        }
        // get up again to not collide with the object
        else{
          instructions.position[2] = (obj_height + safe_heigth);
          instructions.move_gripper = false;
          instructions.gripper_diameter = opened_gripper;
          set_rotations(1, 0, 0, 0, 1, 0, 0, 0, 1);

          pub_instructions.publish(instructions);
        }

        loop_rate.sleep();
      }
    }

  }
  return 0;
}