# Fondamenti di Rbotica - Gruppo W

This repository contains all the sources created by the group W during the course "Fondamenti di Robotica", that took place during the accademic year 2022/2023 in University of Trento.

Open **Project-1_Robot-Robotic_Manipulator.pdf** to have more details about the project.
<br><br>

# How to use
<br>

The repo consist in a ROS 1 package that needs to be integrated with the Locosim repo, so first follow the instructions provided in the
[Locosim repo](https://github.com/mfocchi/locosim) and then download our package in the /lab_exercise folder:

<br>

**NOTE:** We assume the ros_ws folder is located in your home folder

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises
git clone https://github.com/SergioBrodesco/FdR-groupW.git
```

<br>

Some files need to be included in the Locosim environment, so first go in the locosim_addOns folder first copy all the mesh and custom material files

```
cd locosim_addOns/models
cp * ~/ros_ws/src/locosim/ros_impedance_controller/worlds/models
```
<br>
copy also the custom world files

```
cd ../worlds
cp * ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```

<br>
compile and update the packages list

```
cd ~/ros_ws
catkin_make install
source ~/.bashrc
```

<br>

## Yolov5 model
<br>
What we still miss is a local model of Yolov5 and a set of trained weights...

To download the yolov5 model you just need to follow the [Ultralytics](https://github.com/ultralytics/yolov5) guidelines

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises/FdR-groupW/scripts
git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -r requirements.txt  # install
```

<br>

if you have a local set of weigths you can copy it in the /scripts folder as well

<br>

### Modify image_processor.py

<br>

some code lines of /scripts/image_processr.py containing absolute paths to your yolov5 model, mesh files and weights needs to be modified in order to work properly...

copy the absolute path to your model here:
https://github.com/SergioBrodesco/FdR-groupW/blob/849b3415a4917d88b1c121c6b76795be60176be6/scripts/image_processor.py#L534
<br>
copy the absolute path to your weights here:
https://github.com/SergioBrodesco/FdR-groupW/blob/849b3415a4917d88b1c121c6b76795be60176be6/scripts/image_processor.py#L535
<br>
and finally the absolute path to the 3D mesh that you've already downloaded with the repo (located in /scripts/models)
https://github.com/SergioBrodesco/FdR-groupW/blob/849b3415a4917d88b1c121c6b76795be60176be6/scripts/image_processor.py#L540
<br>

now recompile
```
cd ~/ros_ws
catkin_make install
```

<br>

everything should be set properly

<br>

# Testing

First you need to change the world file with one of your choise... we suggest as first test the "my_world.world" file, to do that edit /ur5_generic.py in the following line, and specify '<your_world_name.world>' instead
https://github.com/mfocchi/robot_control/blob/25569e7ad103d58ae0b6095f7ccd570975b36218/lab_exercises/lab_palopoli/ur5_generic.py#L75
<br>

To run the simulation first open a new terminal and run the /ur5_generic.py script, this will initialize all the basic nodes and the Gazebo enviroment
```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli
python3 ur5_generic.py
```
<br>

After the Homing procedure of the robotic arm is accomplished run our 3 custom nodes, *each in a separate terminal*

```
rosrun lab_group_w motion_processor
```
```
rosrun lab_group_w task_planner
```
```
rosrun lab_group_w image_processor
```

<br>

That's it! You should see the simulation running!

<br>



