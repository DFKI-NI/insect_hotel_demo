Insect Hotel Demo
=============

Real robot demo
---------------

Start up the robot according to the (DFKI internal)
[instructions on the wiki](https://git.ni.dfki.de/mobipick/documentation/-/wikis/starting-up-the-robot),
then  on the robot launch:


```bash
roslaunch mobipick_bringup mobipick_bringup_both.launch  # already part of the startup instructions
roslaunch pbr_dope dope.launch
roslaunch insect_hotel_bringup bringup.launch
```

On the Jetson Orin launch the camera driver and object detection:

```bash
roslaunch yolo6d_cpp yolo6d_with_cameras.launch
```

To start the demo, launch on the robot:

```bash
roslaunch insect_hotel_bringup intention_recognition.launch
```

Robot Simulation
------------------

Launches the robot in Gazebo with simulated worker cameras.

```bash
roslaunch insect_hotel_bringup demo_sim.launch
roslaunch insect_hotel_bringup intention_recognition.launch simulation:=true
```

Plan visualization
------------------

Install and source the
[dot_graph_visualization](https://github.com/DFKI-NI/dot_graph_visualization)
rqt plugin, then call it with:

```bash
rqt --standalone dot_graph_visualization
```
