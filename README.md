# Fondamenti di Robotica - Gruppo W

This repository contains all the sources created by the group W during the course "Fondamenti di Robotica", that took place during the accademic year 2022/2023 in University of Trento.

Open [**Project-1_Robot-Robotic_Manipulator.pdf**](https://github.com/SergioBrodesco/FdR-groupW/blob/master/Project-1_Robot-Robotic_Manipulator.pdf) to have more details about the project.
<br><br>

![Alt text](https://github.com/SergioBrodesco/FdR-groupW/blob/master/media/Arm_alone.jpg)

<br>

If you want to read the **Doxygen Documentation** for the code, check the generated PDF doc [here](https://github.com/SergioBrodesco/FdR-groupW/blob/master/docs/latex/refman.pdf).

<br>

Alternatively, you can find the html version of the documentation after completing the setup explained [here](https://github.com/SergioBrodesco/FdR-groupW/blob/master/README.md#how-to-use).
<br>
To view the html page, you need to open the **index.html** file inside the **FdR-groupW/docs/html** folder

<br>

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises/FdR-groupW/docs/html
xdg-open index.html
```

<br>

<br>

# How to use
<br>

The repo consist of a ROS 1 package that needs to be integrated with the Locosim repo, so first follow the instructions provided in the
[Locosim repo](https://github.com/mfocchi/locosim) and then download our package in the /lab_exercise folder:

<br>

**NOTE:** We assume the ros_ws folder is located in your home folder

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises
git clone https://github.com/SergioBrodesco/FdR-groupW.git
```

<br>

Some files need to be included in the Locosim environment, so first go in the locosim_addOns folder and copy all the mesh and custom material files

<br>

```
cd locosim_addOns/models
cp -r * ~/ros_ws/src/locosim/ros_impedance_controller/worlds/models
```
<br>

copy also the custom world files

<br>

```
cd ../worlds
cp * ~/ros_ws/src/locosim/ros_impedance_controller/worlds
```

<br>

compile and update the packages list

<br>

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

some code lines of /scripts/image_processr.py containing absolute paths to your yolov5 model, mesh files and weights need to be modified in order to work properly...

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

First you need to change the world file with one of your choice... we suggest to start with "my_world.world", to do that edit /ur5_generic.py at the following line, and specify '<your_world_name.world>' instead
<br>

https://github.com/mfocchi/robot_control/blob/25569e7ad103d58ae0b6095f7ccd570975b36218/lab_exercises/lab_palopoli/ur5_generic.py#L75

<br>

If you want to use the simulated robot on gazebo, you need to set the real_robot flag as False at this line:

<br>

https://github.com/mfocchi/robot_control/blob/25569e7ad103d58ae0b6095f7ccd570975b36218/lab_exercises/lab_palopoli/params.py#LL42C44-L42C44

<br>

You can also change between the soft and rigid gripper by modifiyng the soft_gripper flag:

<br>

https://github.com/mfocchi/robot_control/blob/25569e7ad103d58ae0b6095f7ccd570975b36218/lab_exercises/lab_palopoli/params.py#LL45C9-L45C9

<br>

Note that in order to use the gripper in simulation you also need to set gripper_sim as True at this line:

<br>

https://github.com/mfocchi/robot_control/blob/25569e7ad103d58ae0b6095f7ccd570975b36218/lab_exercises/lab_palopoli/params.py#LL44C15-L44C15

<br>

To run the simulation first open a new terminal and run the /ur5_generic.py script, this will initialize all the basic nodes and the Gazebo enviroment

<br>

```
cd ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli
python3 -i ur5_generic.py
```

<br>

Otherwise you can also run the ur5_generic.py script using the [pycharm IDE](https://github.com/mfocchi/locosim#running-the-software-from-python-ide-pycharm) if you have it installed and working for locosim.

<br><br>

After the Homing procedure of the robotic arm is accomplished run our 3 custom nodes, **each in a separate terminal**

<br>

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
<br>

# Credits

* Luca Cazzola - Università degli studi di Trento (Unitn), Trento – Italy
  <br> luca.cazzola-1@studenti.unitn.it

* Dennis Cattoni - Università degli studi di Trento (Unitn), Trento – Italy
  <br> dennis.cattoni@studenti.unitn.it

* Sergio Brodesco - Università degli studi di Trento (Unitn), Trento – Italy
  <br> sergio.brodesco@studenti.unitn.it


