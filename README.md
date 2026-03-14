# RoboticsLab 2025/2026
## Homework 3: Fly your drone
-------------
# Overview

The homework consists of three main parts:

1. **Custom UAV creation** for PX4 + Gazebo
2. **Modification of the `force_land` node**
3. **Development of an offboard trajectory planner** with at least 7 waypoints and no intermediate stops

---

## Repository structure

```text
HW3/
├── DDS_run.sh
├── force_land/
├── read_rpy/
├── offboard_rl/
├── PX4-Autopilot/
│   ├── Tools/simulation/gz/models/hw3_drone/
│   │   ├── model.config
│   │   ├── model.sdf
│   │   ├── meshes/
│   │   └── materials/
│   ├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   │   └── 9000_gz_hw3_drone
│   └── src/modules/uxrce_dds_client/
│       └── dds_topics.yaml
└── plots/
```
## Important note

This repository contains the **files modified or developed for the homework**, and is intended as a **delivery repository**.

It is **not a full standalone PX4 workspace**.

To run the project, a complete environment is required, including:

-   **ROS 2 Humble**
-   **Gazebo Harmonic**
-   **QGroundControl**
-   **Micro XRCE-DDS Agent**
-   a complete **PX4-Autopilot v1.16** repository
-   a complete **px4\_msgs** repository

* * *

## Suggested workspace layout

```
~/ros2_ws/src/  
├── HW3/  
│   ├── DDS_run.sh  
│   ├── force_land/  
│   ├── read_rpy/  
│   ├── offboard_rl/  
│   └── PX4-Autopilot/  
└── px4_msgs/
```

* * *

## External dependencies

### Clone `px4_msgs`
```bash
cd ~/ros2_ws/src  
git clone https://github.com/PX4/px4_msgs.git  
cd px4_msgs  
git checkout release/1.16
```

### Full PX4 installation

A full **PX4-Autopilot v1.16** installation is required.

The files contained in this repository must be copied into the corresponding paths of a complete PX4 tree.

* * *

## Build ROS 2 packages
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
colcon build --packages-select px4_msgs read_rpy force_land offboard_rl  
source install/setup.bash
```

* * *

# 1\. Custom UAV

## 1.a Custom model and airframe

The custom drone is defined by:

-   the Gazebo model folder:
    
```
PX4-Autopilot/Tools/simulation/gz/models/hw3_drone/
```
-   the custom airframe:
```
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/9000_gz_hw3_drone
```
The vehicle is based on the PX4 x500 structure, with a custom body mesh and updated rotor coordinates.

## 1.b Actuator outputs validation

To visualize actuator outputs in ROS 2, the following topic was added to:
```
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```
``` YAML
- topic: /fmu/out/actuator_outputs  
  type: px4_msgs::msg::ActuatorOutputs
```

### Launch sequence

#### 1\. Start Micro XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```
#### 2\. Start PX4 + Gazebo
```bash
cd ~/ros2_ws/src/PX4-Autopilot-full  
make px4_sitl gz_hw3_drone
```
#### 3\. Start QGroundControl
```bash
~/Downloads/QGroundControl-x86_64.AppImage
```
#### 4\. Start DDS bridge (alternative script)
```bash
cd ~/ros2_ws/src/HW3  
chmod +x DDS_run.sh  
./DDS_run.sh
```
#### 5\. Start PlotJuggler
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
source install/setup.bash  
ros2 run plotjuggler plotjuggler
```
### Tested flight pattern

The UAV was tested in **Position mode** with the following flight sequence:

-   takeoff
-   short straight flight segment
-   360° yaw rotation around the vertical axis
    
The corresponding actuator outputs were visualized in PlotJuggler.

* * *

# 2\. Modified `force_land` node

The `force_land` node was modified so that:

-   if the UAV goes above the altitude threshold (**20 m**), automatic landing is triggered;
-   if the pilot retakes control before landing is completed, the forced landing is cancelled; 
-   if the UAV goes above the threshold again, forced landing is **not triggered a second time**.
  
The implementation uses:

-   `VehicleLandDetected`
    
-   `VehicleControlMode`
    
## Run the node
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
source install/setup.bash  
ros2 run force_land force_land
```
## Record the validation bag
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
source install/setup.bash  
ros2 bag record -o force_land_altitude /fmu/out/manual_control_setpoint /fmu/out/vehicle_local_position
```
## Validation scenario

The test sequence is:

1.  takeoff
2.  climb above **20 m**    
3.  automatic landing starts 
4.  pilot retakes control during descent
5.  UAV climbs again above the threshold 
6.  forced landing is **not triggered again** 

## Plotting notes

The topic `/fmu/out/vehicle_local_position` is expressed in **NED**, therefore altitude in **ENU** is obtained as:
```
altitude_enu = -z
```
The final validation plot includes:

-   altitude  
-   manual throttle setpoint 

* * *

# 3\. Offboard trajectory planner

A new node called `trajectory_planner` was developed starting from the `go_to_point` implementation.

The planner uses:

-   **7 waypoints**
-   waypoint yaw setpoints 
-   waypoint durations 
-   smooth transition between segments 
-   no stop at intermediate waypoints 
-   zero velocity only at the final waypoint  

## Run the planner
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
source install/setup.bash  
ros2 run offboard_rl trajectory_planner
```
## Record the trajectory bag
```bash
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
source install/setup.bash  
ros2 bag record -o trajectory_plots /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude
```
## Waypoint sequence
```bash
{   0.0,   0.0, -3.0,   0.0,   4.0 }  
{  10.0,   0.0, -5.0,   0.0,   5.0 }  
{  10.0,  10.0, -6.0,  45.0,   5.0 }  
{   0.0,  10.0, -4.0,  90.0,   5.0 }  
{  -5.0,   5.0, -7.0, 135.0,   6.0 }  
{   0.0,   0.0, -5.0, 180.0,   6.0 }  
{   0.0,   0.0,  0.0,   0.0,   5.0 }
```
## Expected plots

The final validation includes:

-   trajectory on the **xy plane** 
-   altitude on the **z axis** in ENU   
-   **yaw**
-   velocity on **x, y, z**
-   acceleration on **x, y, z** 
-   velocity magnitude
-   acceleration magnitude

* * *

## Plotting notes

### Altitude

Since `vehicle_local_position` is expressed in NED:
```
altitude_enu = -z
```
### Yaw extraction

Yaw is obtained from `/fmu/out/vehicle_attitude` quaternion using a custom series in PlotJuggler.

### XY trajectory

If XY plotting is not available directly in PlotJuggler, the trajectory on the xy plane can be exported separately from the rosbag.

* * *

## Main ROS 2 executables

### force\_land
```bash
ros2 run force_land force_land
```
### trajectory\_planner
```bash
ros2 run offboard_rl trajectory_planner
```
### PlotJuggler
```bash
ros2 run plotjuggler plotjuggler
```
* * *

## Notes

-   This repository is intended as a **homework delivery repository**.
-   It contains the files developed and modified for the assignment.   
-   It assumes the presence of a complete PX4 and ROS 2 development environment.   
-   Some mesh references may still rely on resources available in a standard PX4 installation.
