# Unity-ROS Connection Test Package

This package demonstrates and tests the connection between Unity and ROS by integrating the pose of a sphere and joint states of a robot in Unity with trajectory planning in ROS. The goal is to establish seamless communication between the Unity environment and ROS for robotic motion planning and visualization. This connection will enable more complex projects and research in the future.

## Features
- Publishes the **pose of a sphere** and **joint states** of a robot from Unity to ROS.
- Subscribes to the Unity-published data and calculates a trajectory using the MoveIt planning interface.
- Sends the calculated trajectory back to Unity to move the robot towards the sphere's position (one joint state at a time).
- Includes a launch script to start the MoveIt `move_group` instance, Gazebo simulation, and RViz visualization of the robot.

## Prerequisites

### Software Requirements
- **Unity**: Ensure Unity `v2020.3.11f1` is installed on your system.
- **ROS Noetic (or compatible version)**: The package is designed for ROS Noetic.
- **ros_tcp_endpoint**: This package allows Unity to communicate with ROS through TCP. Install it as described below.

### Install Dependencies
To install all the package dependencies, run the following command in your ROS workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```
# Unity Project

The example Unity project is located at:

`include/oculus/UR5e_Test/Assets`

The Unity scene to use is called **Ros Connection Test**.

## Unity Setup

1. Install the **ROS-TCP-Connector** in your Unity project. This allows Unity to communicate with ROS over TCP. Ensure that it is properly configured to connect with your ROS master.
2. Ensure that the Unity project is set up to communicate with the ROS topics published by the nodes in this package.

## Nodes

### 1. unity_listener

**Purpose:** Logs the sphere's pose and the robot's joint states from Unity once the "Publish" button is pressed in the Unity scene.

**Published Topics:**
- `/weld_pose` (geometry_msgs/Pose): Pose of the sphere in Unity.
- `/ur_joint_states` (sensor_msgs/JointState): Current joint states of the robot in Unity.

### 2. Robot Sphere Follower

**Purpose:** Subscribes to the sphere's pose and robot's joint states, computes a trajectory using MoveIt, and sends the trajectory back to Unity.

**Subscribed Topics:**
- `/sphere_pose`: Pose of the sphere in Unity.
- `/ur_joint_state`: Current joint states of the robot in Unity.

**Published Topics:**
- `/unity_joint_commands` (sensor_msgs/JointState): Joint States for the robot to move towards the sphere.

## Launch Instructions

### Launch Commands for Testing Connection

1. **Start the Listening Node**

    ```bash
    roslaunch weldingrobot unity_listener
    ```

    This will:
    - Start logging the pose and joint states published from Unity.

### Launch Commands for Robot Sphere Following

1. **Start the MoveIt instance and Gazebo with RViz:**

    ```bash
    roslaunch weldingrobot ur5e_lab.gazebo.launch
    ```

    This will:
    - Start the MoveIt move_group.
    - Launch Gazebo with the robot model.
    - Open RViz for visualization.

2. **Run the Trajectory Planning Node:**

    ```bash
    rosrun weldingrobot unity_sphere_follow
    ```

### Unity Setup:

1. Open Unity and load the example project at `include/oculus/UR5e_Test/Assets`.
2. Open the **Ros Connection Test** scene.
3. Press the "Publish" button to start sending pose and joint state data to ROS.

## Visual Feedback

- **RViz:** Displays the robot and its planned trajectory in real-time.
- **Gazebo:** Simulates the robot's movements based on the planned trajectory.
- **Unity:** Reflects the robot's pose updates and moves the robot towards the sphere.

## Future Enhancements

- Move trajectory segmentation inside unity, instead of publishing joints individually.
- Jogging of end-effector movement using moveit Servo for real-time sphere following.
- Add error handling for communication delays or connection failures.
- Extend support to more complex environments and robotic configurations.
- Improve trajectory calculation with constraints like obstacles and time optimization.

## Troubleshooting

If you encounter issues:

- Ensure that Unity's TCP connector is properly configured with the ROS master URI and IP address.
- Verify that all required ROS topics are being published and subscribed correctly.
- Check for any errors in the logs of the Unity and ROS nodes.
