#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <iostream>
#include <thread>
#include <mutex>

geometry_msgs::Pose incoming_pose;
sensor_msgs::JointState unity_joints;

std::mutex mtx;
bool pose_received = false;
bool joint_states_received = false;

void unityPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    ROS_INFO("NEW POSES RECEIVED...");
    mtx.lock();
    incoming_pose = *msg;
    pose_received = true;
    mtx.unlock();
}

void unityJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("NEW JOINTS STATES RECEIVED...");
    unity_joints = *msg;
    joint_states_received = true;
}

bool computeTrajectory(moveit::planning_interface::MoveGroupInterface& move_group, 
                       const geometry_msgs::Pose& target_pose, 
                       const sensor_msgs::JointState& current_joint_states, 
                       moveit::planning_interface::MoveGroupInterface::Plan& plan) 
{
    // Lock the mutex if necessary
    mtx.lock();

    // Set the robot's current joint state from the joint states message
    robot_state::RobotState start_state(*move_group.getCurrentState());
    start_state.setVariablePositions(current_joint_states.name, current_joint_states.position);
    move_group.setStartState(start_state);

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan to the target pose
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Unlock the mutex if it was locked
    mtx.unlock();

    if (success) {
        ROS_INFO("Trajectory planning succeeded.");
    } else {
        ROS_WARN("Trajectory planning failed.");
    }
    
    return success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;

    ros::Subscriber target_pose = node_handle.subscribe("weld_pose", 10, unityPoseCallback);
    ros::Subscriber unity_joints_sub = node_handle.subscribe("ur_joint_states", 10, unityJointStatesCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set up the MoveIt! MoveGroup interface for the UR5e robot
    static const std::string PLANNING_GROUP = "manipulator";  // Replace with your robot's planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (ros::ok()) {
        if (pose_received && joint_states_received) {
            // Call the computeTrajectory function with the pose and joint states
            if (computeTrajectory(move_group, incoming_pose, unity_joints, my_plan)) {
                // Execute the plan if desired
                move_group.execute(my_plan);
            } else {
                ROS_WARN("Unable to execute trajectory.");
            }

            // Reset flags after processing
            pose_received = false;
            joint_states_received = false;
        } else {
            ROS_WARN("Waiting for new poses and joint states...");
            ros::Duration(1.0).sleep(); // Wait before checking again
        }
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
