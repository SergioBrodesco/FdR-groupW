cmake_minimum_required(VERSION 2.8.3)
project(lab_group_w)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation)
find_package(Eigen3 3.3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  RobotInstructions.msg
  ImageData.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  lab_group_w
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs
)

## Declare a Python executable
catkin_install_python(PROGRAMS
  scripts/image_processor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(motion_processor

              src/motion_processor.cpp
              src/pos_manager.cpp
              src/trajectory_planner.cpp
              src/utils.cpp
              src/collision_handler.cpp

              kinematic_libs/invDirKinematics.cpp
              kinematic_libs/ur5Jac.cpp
              kinematic_libs/p2pMotionPlan.cpp
              kinematic_libs/invDiffKinematicControlSimCompleteAngleAxis.cpp
)
add_executable(task_planner src/task_planner.cpp)

target_link_libraries(motion_processor ${catkin_LIBRARIES})
add_dependencies(motion_processor ${catkin_EXPORTED_TARGETS})

install(TARGETS motion_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#
#
#
target_link_libraries(task_planner ${catkin_LIBRARIES})
add_dependencies(task_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS task_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)