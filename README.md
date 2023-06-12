# FdR-groupW

This repository contains all the sources created by the group W during the course "Fondamenti di Robotica", that took place during the accademic year 2022/2023 in Trento.

Open "Project-1_Robot-Robotic_Manipulator" to have more details about the project.

# How to use

The repo consist in a ROS 1 package that needs to be integrated with the Locosim repo, so first follow the instructions provided in the [Locosim repo](https://github.com/mfocchi/locosim) and then download our package in the /lab_exercise folder:

**NOTE:** We assume the ros_ws folder is located in your home folder

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises
git clone https://github.com/SergioBrodesco/FdR-groupW.git
```

Some files need to be included in the Locosim environment, so first go in the locosim_addOns folder first copy all the mesh and custom material files

```
cd locosim_addOns/models
cp * ~/ros_ws/src/locosim/ros_impedance_controller/worlds/models
```

copy also the custom world files

```
cd ../worlds
cp * ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```

