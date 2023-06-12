/**
 * @file collision_handler.h
 * @brief Header with the declaration of the functions implemented in collision_handler.cpp
 * 
 */
#ifndef COLLISION_HANDLER_H
#define COLLISION_HANDLER_H

#include "invDirKinematics.h"

using namespace Eigen;

/**
 * @brief Struct containing the information of an object that we want to check collisions with
 * 
 */
struct CollisionBox {
  /// @brief rotation matrix we use to translate a point to the CollisionBox frame
  Eigen::Matrix3d rotation_matrix;
  /// @brief translation vector we use to translate a point to the CollisionBox frame
  Eigen::Vector3d translation;
  /// @brief vector containing length, width and height of the CollisionBox
  Eigen::Vector3d extents;
  /// @brief Collsion Box name
  std::string box_name; 
};

/**
 * @brief Function that checks if a given point is inside of a CollisionBox object
 * 
 * @param point point we'll use to check for collisions
 * @param box object we want to know collisions on
 * @return true if the point is inside the CollisionBox (we have a collision)
 */
bool isPointInsideBox(const Eigen::Vector3d& point, const CollisionBox& box);

/**
 * @brief Function used to create the "fixed" collision boxes (Table, ...) that will not change during the robot motion.\n
 * The collision box for the arm is evaluated for every configuration we check collisions on.
 * @return std::vector<CollisionBox> List of CollisionBox objects
 */
std::vector<CollisionBox> define_world_cBoxes();

/**
 * @brief Get specific joint information (joint_pos, joint_rot, joint_length), knowing the current configuration
 * 
 * @param joint_index index of the joint we want informations about
 * @param config current configuration of the joints
 * @param joint_pos position of the joint with regards to the robot base frame, obtained with the same method as Direct kinematics
 * @param joint_rot rotation of the joint with regards to the robot base frame, obtained with the same method as Direct kinematics
 * @param joint_length length of the joint along the three axis (X, Y, Z)
 */
void get_joint_info (const int joint_index, const VectorXd& config, Vector3d& joint_pos, Matrix3d& joint_rot, Vector3d& joint_length);

/**
 * @brief Function used to get a set of 4 points (vertices of a square) in the same plane (around the robot arm)
 * 
 * @param dimension axis that we want our square to be perpendicular to, so that our square is built around the robot's link
 * @param w length representing half of the square's side
 * @return MatrixXd matrix containing the 4 vertices of the square
 */
MatrixXd get_square_matrix_plane (int dimension, float w);

//std::vector<Vector3d> get_joint_collision_points (const double spacing, const Vector3d& joint_pos, const Matrix3d& joint_rot, const Vector3d& joint_length);
/**
 * @brief Get the points that we use to evaluate collisions for a specific joint
 * 
 * @param square_spacing spacing between each set of 4 points along the robot's link
 * @param box_extents distance (divided by 2) that the 4 points have from eachother 
 * @param joint_pos position of the joint
 * @param joint_rot rotation of the joint
 * @param joint_length length of the joint along the 3 dimensions (X, Y, Z)
 * @return std::vector<Vector3d> vector containing a set of points (built around the arm) that we'll use to check for collisions
 */
std::vector<Vector3d> get_joint_collision_points (const double square_spacing, const double box_extents ,const Vector3d& joint_pos, const Matrix3d& joint_rot, const Vector3d& joint_length);

/**
 * @brief Function that checks for collisions with the world or the robot itself (by building CollisionBox objects around the arm's links).\n
 * This control is done every 10 configurations
 * 
 * @param joint_configs configurations of the joints during a computed trajectory
 * @param conf_index index of the final configuration we are evaluating
 * @return true if the trajectory doesn't have any collisions
 */
bool checkCollisions(const MatrixXd& joint_configs, int conf_index);

#endif