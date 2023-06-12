/**
 * @file trajectory_planner.cpp
 * @brief Implementation of the functions we use to compute a trajectory for the robot arm.
 * 
 * 
 */
#include "../include/trajectory_planner.h"

std::array<bool,8> exclude_invalid_confs (const MatrixXd& Th, const Vector3d& goal_point){

  std::array<bool,8> valid_config = {true, true, true, true, true, true, true, true};

  float x_upper_limit = 1;
  float x_lower_limit = -0.02;

  float y_upper_limit = 0.8;
  float y_lower_limit = 0.155;

  float z_upper_limit = 1.75;
  float z_lower_limit = 0.866;

  for (int i = 0; i < Th.rows(); i++){
    for (int k = 0; k < 6 && valid_config[i]; k++){
      // Exclude configurations returning NaN
      if(Th(i,k) != Th(i,k)){
        valid_config[i] = false;
      }
    }
    auto direct_res = ur5Direct(Th.row(i));
    Vector3d pos_f = std::get<0>(direct_res);

    Vector3d diff = goal_point - pos_f;
    float tollerance = 0.015;
    // Exclude if the configuration doesn't reach the object with a little bit of tollerance
    if(fabs(diff(0)) > tollerance || fabs(diff(1)) > tollerance && fabs(diff(2)) > tollerance){
      valid_config[i] = false;
    }
    // Exclude if the goal point is outside the table for some reason
    else{
      Vector3d goal_point_world = R_t_W_transform(goal_point);

      if (goal_point_world(0) > x_upper_limit || goal_point_world(0) < x_lower_limit || goal_point_world(1) > y_upper_limit || goal_point_world(1) < y_lower_limit || goal_point_world(2) > z_upper_limit || goal_point_world(2) < z_lower_limit){
        valid_config[i] = false;
      }
    }
  }

  return valid_config;
}

std::array<int, 8> sort_confs (const MatrixXd& Th, const VectorXd& q_0, const std::array<bool, 8>& valid_config){
  
  std::array<int, 8> conf_index_order = {0, 1, 2, 3, 4, 5, 6, 7};
  VectorXd q_f (6);

  double space_diff [8] = { HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL };
  for(int i = 0; i < 8; i++){
    if(valid_config[i]){
      q_f = Th.row(i);

      // Compute difference between starting conf and ending conf for each joint
      double total_diff=0;
      for(int k=0; k<6; k++) {
        total_diff += abs(q_f(k) - q_0(k));
      }

      conf_index_order[i] =  i;
      space_diff[i] = total_diff;
    }
  }

  // Sort to get the order
  // (Using Bubble sort)
  for (int i = 0; i < 7; i++){
    for(int k = i+1; k < 8; k++){
      if (space_diff[k] < space_diff[i]){

        double tmp_index = conf_index_order[i];
        int tmp_value = space_diff[i];

        conf_index_order[i] = conf_index_order[k];
        space_diff[i] = space_diff[k];

        space_diff[k] = tmp_value;
        conf_index_order[k] = tmp_index;
      }
    }
  }

  return conf_index_order;
}

double find_optimal_maxT (const VectorXd& q_0 , const VectorXd& q_f, const double scaling_factor, const double minimum_time){

  double max_shift = -HUGE_VAL;
  VectorXd diff = q_f - q_0;

  for (int i = 0; i < diff.size(); i++){
    if(fabs(diff(i)) > max_shift){
      max_shift = fabs(diff(i));
    }
  }

  // Max velocity for a joint obtained from the "joint_limits.yaml" file
  double max_velocity = M_PI;
  return minimum_time + (max_shift/(scaling_factor * max_velocity));
}


// Iterate over Joint configurations to find the best:
// Th : joint config matrix from inverse kinematics
// q_0 : starting joint config

MatrixXd get_best_config(const MatrixXd& Th, const VectorXd& q_0, const Vector3d& goal_point, const double dt) {
  // Setup variables
  int DOF = 6;
  int num_steps;
  int num_cols = DOF + 1;

  double minT = 0;
  double maxT = 0;
  
  VectorXd q_f (6);
  Vector3d P_r;

  //
  //
  // Compute current pose of the wrist
  std::tuple<Vector3d, Matrix3d> start_config = ur5Direct(q_0);
  Vector3d pos_0 = std::get<0>(start_config);
  Matrix3d rot_0 = std::get<1>(start_config);

  // Array containing which configurations are available
  std::array<bool,8> valid_config = exclude_invalid_confs(Th, goal_point);

  // joint configurations are ordered based on the one that has the smallest difference in joint angles from the starting configuration
  std::array<int, 8> conf_index_order = sort_confs(Th, q_0, valid_config);

  // Iterate following the order defined previously untill a valid configuration is found.
  // A valid configuration is one that doesn't collide with the table or the rebot itself
  int best = -1;
  float scaling_factor = 0.5;
  float minimum_time = 0.8;

  for (int i=0; i < 8; i++) {
    int j_index = conf_index_order[i];

    if (valid_config[j_index]){
      q_f = Th.row(j_index);

      maxT = find_optimal_maxT (q_0 , q_f, scaling_factor, minimum_time);
      std::cout << std::endl << "MaxT : " << maxT << std::endl;
      num_steps = (int)((maxT - minT) / dt) + 1;

      MatrixXd Bh (num_steps, num_cols);
      MatrixXd xd (num_steps, 4);
      MatrixXd phid (num_steps, 4);

      // Bh will contain the set of configurations representing the trajectory

      auto result = p2pMotionPlan(q_0, q_f, minT, maxT, dt, num_steps);
      Bh = std::get<0>(result);
      xd = std::get<1>(result);
      phid = std::get<2>(result);

      auto result_2 =  invDiffKinematicControlSimCompleteAngleAxis(xd, phid , q_0, q_f, minT, maxT, dt);
      Bh = std::get<0>(result_2);
      xd = std::get<1>(result_2);
      phid = std::get<2>(result_2);

      // Consider trajectory only if there are no collisions
      valid_config[j_index] = checkCollisions(Bh, j_index);

      if(valid_config[j_index]){
        std::cout << "Configuration [" << j_index << "] : VALID" << std::endl << std::endl;
        best = j_index;
        break;
      }
    }
    else{
      std::cout << "Configuration [" << j_index << "] NOT VALID" << std::endl;
      std::cout << "Target destination isn't allowed" << std::endl << std::endl;
    }
  }

  // Return the best valid configuration 
  
  if( best == -1){
    std::cout << "NO VALID CONFIGURATION FOUND" << std::endl;
    
    // Close program since there's no other option
    exit(EXIT_FAILURE);
  }
  else{
    std::cout << "BEST CONFIGURATION : " << best << std::endl;
  }
  std::cout << std::endl << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+" << std::endl;
  
  MatrixXd Bh (num_steps, num_cols);
  MatrixXd xd (num_steps, 4);
  MatrixXd phid (num_steps, 4);

  q_f = Th.row(best);

  auto result = p2pMotionPlan(q_0, q_f, minT, maxT, dt, num_steps);

  Bh = std::get<0>(result);
  xd = std::get<1>(result);
  phid = std::get<2>(result);
  
  auto result_2 =  invDiffKinematicControlSimCompleteAngleAxis(xd, phid , q_0, q_f, minT, maxT, dt);
  Bh = std::get<0>(result_2);
  xd = std::get<1>(result_2);
  phid = std::get<2>(result_2);

  return Bh;
} 