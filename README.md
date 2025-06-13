# ScLERP_UserGuidance

**Point-to-Point Path Planning Based on ScLERP and User Guidance**

This project implements point-to-point path planning for the Franka Emika Panda robot using **ScLERP (Screw Linear Interpolation of Dual Quaternions)** and user-guided demonstrations. It supports both real robot execution and Gazebo simulation.

- Smooth trajectory interpolation based on dual quaternions (ScLERP)
- User-guided path collection through demonstration
- Joint velocity control interface
- Compatible with both real robot and simulation
- Complete pipeline for data recording and processing to CSV

---

## Installation
### 1. Python Dependencies
https://github.com/Achllle/dual_quaternions
```bash
pip install dual_quaternions
```
https://github.com/petercorke/robotics-toolbox-python
```bash
pip3 install roboticstoolbox-python
```
### 2. ROS Dependencies (ROS Noetic)

Ensure you are using **ROS Noetic** with franka ROS and moveit


##  Data Collection
Navigate to the `scripts/data` directory and run the following to record demonstration data:

```bash
rosbag record -O demo_joint_states.bag /joint_states
```

Convert the rosbag to CSV format:

```bash
rostopic echo -p /joint_states -b demo_joint_states.bag > joint_states_raw.csv
python3 bag2csv.py
```

---

## Launch Instructions

### Real Robot (replace IP with your robot's IP address):

```bash
roslaunch panda_moveit_config move_group.launch
roslaunch franka_example_controllers joint_velocity_example_controller.launch robot_ip:=192.168.3.108
```

### Simulation (Gazebo):

```bash
roslaunch franka_gazebo panda.launch controller:=joint_velocity_example_controller rviz:=true
```


### Using TSIA to publish command (velocity controll) :

```bash
python3 tsia_vel_controller.py --visualize --csv joint_trajectory.csv
```
