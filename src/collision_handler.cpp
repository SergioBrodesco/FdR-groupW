/**
 * @file collision_handler.cpp
 * @brief Implementation of the functions used for collision control.
 * 
 * 
 */
#include "../include/collision_handler.h"

void get_joint_info (const int joint_index, const VectorXd& config, Vector3d& joint_pos, Matrix3d& joint_rot, Vector3d& joint_length){
  
  Matrix4d dh;
  switch (joint_index){
    case 0 : dh = Matrix4d::Identity();                                                       break;
    case 1 : dh = T10(config(0));                                                             break;
    case 2 : dh = T10(config(0))*T21(config(1));                                              break;
    case 3 : dh = T10(config(0))*T21(config(1))*T32(config(2));                               break;
    case 4 : dh = T10(config(0))*T21(config(1))*T32(config(2))*T43(config(3));                break;
    case 5 : dh = T10(config(0))*T21(config(1))*T32(config(2))*T43(config(3))*T54(config(4)); break;
  }

  // Get selected joint joint traslation and rotation vector respect to robot base frame
  joint_pos << dh(0,3), dh(1,3), dh(2,3);

  joint_rot << dh(0,0), dh(0,1), dh(0,2),
               dh(1,0), dh(1,1), dh(1,2),
               dh(2,0), dh(2,1), dh(2,2);

  // Get specific joint parameters 
  switch (joint_index)
  {
    case 0 : joint_length.z() = 0.1625;            break;
    case 1 : joint_length.y() = 0;                 break;   
    case 2 : joint_length.x() = -0.425;            break;
    case 3 : joint_length.x() = -0.3922;           break;
    case 4 : joint_length.y() = -0.0997;           break;
    case 5 : joint_length.y() = (0.0996 + 0.1085); break;
  }

  // N.B:
  // joint 1 length is set to Zero because the segment is already covered by
  // joints 0 and 2 in terms of collisions, adding it would be overhead 
}

MatrixXd get_square_matrix_plane (int dimension, float w){

  MatrixXd square (4,3);

  switch(dimension){
    case 0 : // Y-Z plane
    square << 0, -w,  w, 
              0,  w,  w,
              0, -w, -w,
              0,  w, -w;
    break;

    case 1 : // X-Z plane
    square << -w, 0,  w,
                w, 0,  w,
              -w, 0, -w,
                w, 0, -w;
    break;

    case 2 : //X-Y plane
    square <<  -w,  w, 0,
                w,  w, 0,
                -w, -w, 0,
                w, -w, 0;
    break;
  }

  return square;
}

std::vector<Vector3d> get_joint_collision_points (const double square_spacing, const double box_extents ,const Vector3d& joint_pos, const Matrix3d& joint_rot, const Vector3d& joint_length){
    
  // Vector containing points that are sensitive to collisions
  std::vector<Vector3d> joint_collision_points;

  // Go through dimension X-Y-Z
  for (int i = 0; i < 3; i++){
    //check if there's a length set along the dimension 'i'
    if(fabs(joint_length(i)) > 0){
      MatrixXd square = get_square_matrix_plane(i, box_extents);
      float offset = joint_length(i);
      int collisionSquares_num = abs((int)((joint_length(i) / square_spacing)));
      
      // Create collision squares along the length of each joint
      for(int k = 0; k < collisionSquares_num; k++){
        for(int j = 0; j < 4; j++){
          Vector3d point = square.row(j).transpose();
          point(i) += offset;
          joint_collision_points.push_back(joint_rot * point + joint_pos);
        }
        
        if(joint_length(i) > 0)
          offset -= square_spacing;
        else
          offset += square_spacing;
      }
    }
  }
  return joint_collision_points;
}

bool violates_joint_limits (const VectorXd& conf){
  if(conf(0) < -6.14 || conf(0) > 6.14){
    std::cout << "joint : 0 violates limits: " << conf(0) << std::endl;
     return true;
  }
  if (conf(1) < -3.14 || conf(1) > 0){
    std::cout << "joint : 1 violates limits: " << conf(1) << std::endl;
     return true;
  }
  if (conf(2) < -3.14 || conf(2) > 3.14){
    std::cout << "joint : 2 violates limits: " << conf(2) << std::endl;
     return true;
  }
  if (conf(3) < -6.28 || conf(3) > 6.28){
    std::cout << "joint : 3 violates limits: " << conf(3) << std::endl;
     return true;
  }
  if (conf(4) < -6.28 || conf(4) > 6.28){
    std::cout << "joint : 4 violates limits: " << conf(4) << std::endl;
     return true;
  }
  if (conf(5) < -6.28 || conf(5) > 6.28){
    std::cout << "joint : 5 violates limits: " << conf(5) << std::endl;
     return true;
  }
  return false;
}

std::vector<CollisionBox> define_world_cBoxes (){
  
  // Table Surface
  CollisionBox table_surface;
  table_surface.box_name = "Table surface";
  table_surface.translation << ((-0.02 + 1.0) / 2) , ((0.8 + 0.155) / 2), (0.866 / 2);
  table_surface.translation = W_t_R_transform(table_surface.translation);
  table_surface.rotation_matrix = Matrix3d::Identity();
  table_surface.extents << (1.02 / 2), ((0.8-0.155)/2), (0.866/2);

  // Table Back
  CollisionBox table_back;
  table_back.box_name = "Table back";
  table_back.translation << ((- 0.02 + 1.0) / 2), ((0.155 + 0.005) / 2), (1.015 / 2);
  table_back.translation = W_t_R_transform(table_back.translation);
  table_back.rotation_matrix = Matrix3d::Identity();
  table_back.extents << (1.02/2), ((0.155-0.005)/2), (1.015/2);


  CollisionBox wall;
  wall.box_name = "Wall";
  wall.translation << ((0 + 0.98) / 2), ((-0.05 -0.35) / 2), (1.85 / 2);
  wall.translation = W_t_R_transform(wall.translation);
  wall.rotation_matrix = Matrix3d::Identity();
  wall.extents << (0.98/2) , (0.30/2) , (1.85/2);

  // Right Leg
  //CollisionBox rigth_leg;
  //rigth_leg.box_name = "Rigth leg";
  //rigth_leg.translation << ((0 + 0.05) / 2), ((+0.05 - 0.05) / 2), (1.85 / 2);
  //rigth_leg.translation = W_t_R_transform(rigth_leg.translation);
  //rigth_leg.rotation_matrix = Matrix3d::Identity();
  //rigth_leg.extents << (0.05/2) , (0.10/2) , (1.85/2);

  // Left Leg
  //CollisionBox left_leg;
  //left_leg.box_name = "Left leg";
  //left_leg.translation << ((0.93 + 0.98) / 2), ((+0.05 - 0.05) / 2), (1.85 / 2);
  //left_leg.translation = W_t_R_transform(left_leg.translation);
  //left_leg.rotation_matrix = Matrix3d::Identity();
  //left_leg.extents << (0.05/2) , (0.10/2) , (1.85/2);

  std::vector<CollisionBox> world_cBoxes;
  world_cBoxes.push_back(table_surface);
  world_cBoxes.push_back(table_back);
  world_cBoxes.push_back(wall);

  //world_cBoxes.push_back(rigth_leg);
  //world_cBoxes.push_back(left_leg);

  return world_cBoxes;
}

// Perform a point-in-box test for a rotated box
bool isPointInsideBox(const Eigen::Vector3d& point, const CollisionBox& box) {
  Eigen::Vector3d local_point = box.rotation_matrix.transpose() * (point - box.translation);
  return (local_point.array().abs() <= box.extents.array()).all();
}

bool checkCollisions(const MatrixXd& joint_configs, int conf_index) {

  std::vector<CollisionBox> world_cBoxes = define_world_cBoxes();
  
  // Iterate Over each configuration in the Trajectory
  bool valid_conf = true;
  int skips = 10;

  for (int k = 0; k < joint_configs.rows() && valid_conf; k++){

    // Do collision check only each "skips" points
    if (k % skips == 0){

      // Load the configuration excluding the first element
      // ( first element tells the "dt" relative to the configuration ) 
      VectorXd config (6);
      config << joint_configs(k, 1), joint_configs(k, 2), joint_configs(k, 3), joint_configs(k, 4), joint_configs(k, 5), joint_configs(k, 6);
      
      // Discard trajectory if it exeeds joint limits
      if(!violates_joint_limits(config)){

        std::vector <CollisionBox> robot_cBoxes;
        // Iterate over each Joint
        for (int i = 0; i < config.size() && valid_conf; i++) {

          Vector3d joint_pos;
          Matrix3d joint_rot;
          Vector3d joint_length = Vector3d::Zero();

          //populate vectors with the appropriate values
          get_joint_info(i, config, joint_pos, joint_rot, joint_length);

          // collision points displaced in squares of "box_xtents"*2 meters size
          // and are spaced "spacing" meters along the length of the link
          double square_spacing = 0.04;
          double box_extents = 0.07;

          // Vector containing points that are sensitive to collisions
          std::vector<Vector3d> joint_collision_points = get_joint_collision_points(square_spacing, box_extents, joint_pos, joint_rot, joint_length);

          // Define collision box for the joints
          CollisionBox jointBox;
          
          for(int dim = 0; dim < 3; dim++){
            if(joint_length(dim) != 0)
              jointBox.extents(dim) = fabs(joint_length(dim)/2);
            else
              jointBox.extents(dim) = box_extents;
          }
          // Joint 2 has an offset along Z that is not taken into account to compute direct kinematics
          // to have the box placed precisely we need to add the "shoulder offset" to make it match
          if (i == 2)
            joint_length.z() += 0.1333;

          jointBox.translation = joint_rot * (0.5*joint_length) + joint_pos;
          jointBox.rotation_matrix = joint_rot;

          robot_cBoxes.push_back(jointBox);
        
          
          // Check if the collision points violate defined boxes
          for (int j = 0; j < joint_collision_points.size(); j++) {
            // Exclude first 2 joints from collision checks
            if (i >= 2){
              // Check for collisions on the world obstacles (table surface for instance)
              for (int q = 0; q < world_cBoxes.size() && valid_conf; q++){
                if (isPointInsideBox(joint_collision_points[j], world_cBoxes[q])){
                  valid_conf = false;
                  std::cout << "Configuration [" << conf_index << "] : NOT VALID" << std::endl;
                  std::cout << "Trajectory collision : Joint " << i << " - " << world_cBoxes[q].box_name << std::endl << std::endl;
                  break;
                }
              }

              // Self collisions are evaluated only from joint N to its precedent joints excluding N-1. 
              // Since joints are rigid bodies the collision can't accour between direct neighburs N-1 and N+1.
              //
              // Evaluating a collision between N and N+2, N+3 (and so on) is also overhead, because the position of N+2 for instance is
              // determined by the rotation of its predecessors, it makes more sense looking only backwards 
              // for example we're only checking if N+2 goes into N instead of N going into N+2
              //
              // Since Joint 1 has "joint_length" = 0 in all dimensions (because its box blends with the box of 0 and 2) it's like 0 and 2
              // are direct neighbours... so we start evaluating only from joint 3
              if(i >= 3){
                // Check self collisions on previous joint collision boxes
                for(int q = robot_cBoxes.size()-3 && valid_conf; q >= 0; q--){
                  if (isPointInsideBox(joint_collision_points[j], robot_cBoxes[q])){
                    valid_conf = false;
                    std::cout << "Configuration [" << conf_index << "] : NOT VALID" << std::endl;
                    std::cout << "Trajectory collision : joints [" << i << " - " << q << "]" << std::endl << std::endl;
                    break;
                  }
                }
              }
            }
          }
        }
      }
      else{
        valid_conf = false;
        std::cout << "Configuration [" << conf_index << "] : NOT VALID" << std::endl;
        std::cout << "Trajectory exceed joint limits" << std::endl << std::endl;
        break;
      }
    }
  }
  return valid_conf;
}