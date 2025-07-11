# Active Goal Recognition on the Insect Hotel Scenario

This repository contains the source code to replicate the demo scenario described in:

J.C. Saborío, M. Vinci, O. Lima, S. Stock, L. Niecksch, M. Günther, A. Sung, J. Hertzberg and M. Atzmüller. "Uncertainty-Resilient Active Intention Recognition for Robotic Assistants". On Proceedings of ECMR 2025 (to appear).
 
## Video

We are preparing a detailed video showcasing the Mobipick robot in the Insect Hotel domain. A link will be added here as soon as it is ready.

## Installation

ROS Noetic and Python 3 are assumed to be available on your system. [vcs](https://pypi.org/project/vcstool/) and
[wstool](http://wiki.ros.org/wstool) will be installed when needed:

```bash
./install-deps.sh
./build.sh
```

## Setup and Usage

### Real Robot Demo

Start up the robot according to the (DFKI internal)
[instructions on the wiki](https://git.ni.dfki.de/mobipick/documentation/-/wikis/starting-up-the-robot),
then on the robot launch:

```bash
roslaunch mobipick_bringup mobipick_bringup_both.launch  # already part of the startup instructions
roslaunch pbr_dope dope.launch
roslaunch insect_hotel_bringup bringup.launch   # world_config:=insect_hotel   (default)
```

On the Jetson Orin launch the camera driver and object detection:

```bash
roslaunch yolo6d_cpp yolo6d_with_cameras.launch
```

To start the demo, launch on the robot:

```bash
roslaunch insect_hotel_bringup intention_recognition.launch
```

### Robot Simulation

Launches the robot in Gazebo with simulated worker cameras.

```bash
roslaunch insect_hotel_bringup demo_sim.launch   # world_config:=insect_hotel_tables_spread   (default)
roslaunch insect_hotel_bringup intention_recognition.launch simulation:=true
```

### Worker Simulation

The behavior of the worker is simulated by moving insect hotel parts
from the storage area to the assembly area. Inside the
`insect_hotel_bringup/launch/includes/intention_recognition.launch` these
[lines](https://github.com/DFKI-NI/insect_hotel_demo/blob/4d4905c4b31c079af8c827a242ab512e481d5b03/insect_hotel_bringup/launch/includes/intention_recognition.launch#L18-L27)
modify the simulated workers behavior. The following parameters can be changed:

- hotel_type: Specify hotel type 1 or 2, type 1 uses red side panel and dark green back panel, type 2 uses orange side panel and black back panel
- worker_time_between_actions: Time in seconds between taking parts from the storage
- random_worker_actions: If true takes parts from storage randomly, if false takes parts in fixed assembly order
- prob_to_skip_action: Probability for the worker to do nothing and skip an assembly action
- prob_to_take_wrong_part: Probability for the worker to take a wrong part, e.g. a part that does not belong to the insect hotel type that is currently being built
- prob_to_fix_wrong_part: Probability to fix the workers mistakes, if any occured

### Plan Visualization

Install and source the
[dot_graph_visualization](https://github.com/DFKI-NI/dot_graph_visualization)
rqt plugin, then call it with:

```bash
rqt --standalone dot_graph_visualization
```

## Advanced Documentation

### World Configs

This repository defines two so-called `world_config`s (i.e., arrangements of
tables, objects etc.):

1. `insect_hotel`: The regular arrangement of tables in our physical robot lab,
   with the insect hotel parts spread over the tables. Available in reality and
   simulation. This is the default for the real robot demo.
2. `insect_hotel_tables_spread`: The same as above, but the tables are spread
   apart further. Does not reflect the actual table arrangement in the lab, so
   this `world_config` is only available in simulation. This is the default for
   simulation.

The reason for introducing the second `world_config` is the following: In
reality, we use the internal `move_base` instance that is running on the
internal PC of the MiR platform. This `move_base` instance can deal with the
regular, crowded table arrangement. Obviously, we cannot directly use the MiR
PC in simulation, so we had to replicate the `move_base` config in the
`mir_navigation` package for use in simulation. Unfortunately, this config does
not include some proprietary parts of the MiR configuration, so it sometimes
struggles in crowded environments. To work around this issue, we spread the
tables further apart for simulation only.

Switching between the two is done by passing the argument
`world_config:=insect_hotel...` to `bringup.launch` (for running on the real
robot) or `demo_sim.launch` (for simulation).

**TL;DR:** In reality, use `world_config:=insect_hotel`. In simulation, use
`world_config:=insect_hotel_tables_spread`. These are already the defaults.
