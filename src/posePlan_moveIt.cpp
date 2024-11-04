#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include "weldingrobot/moveit_error_handler.h"
#include <iostream>
#include <thread>
#include <mutex>

geometry_msgs::Pose incoming_pose;
sensor_msgs::JointState unity_joints;

std::mutex mtx;
bool pose_received = false;
bool joint_states_received = false;
bool start_ready = false;
bool trajectroy_interupt = false;

void unityPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    ROS_INFO("NEW POSES RECEIVED...");
    mtx.lock();
    incoming_pose = *msg;
    pose_received = true;
    mtx.unlock();
}

void unityJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    ROS_INFO("NEW JOINTS STATES RECEIVED...");
    unity_joints = *msg;
    joint_states_received = true;
}

bool handlePlanError(moveit::core::MoveItErrorCode my_plan, std::string planning)
{
    switch (my_plan.val)
    {
    case moveit::core::MoveItErrorCode::SUCCESS:
        if (planning == "planning")
        {
            ROS_INFO("Planning successful!");
        }
        else
        {
            ROS_INFO("Path Executed Successfully...");
        }
        return true;
    case moveit::core::MoveItErrorCode::PLANNING_FAILED:
        ROS_ERROR("Error: Planning failed.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
        ROS_ERROR("Error: Invalid motion plan.");
        break;
    case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        ROS_ERROR("Error: Motion plan invalidated by environment change.");
        break;
    case moveit::core::MoveItErrorCode::CONTROL_FAILED:
        ROS_ERROR("Error: Control failed.");
        break;
    case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
        ROS_ERROR("Error: Unable to acquire sensor data.");
        break;
    case moveit::core::MoveItErrorCode::TIMED_OUT:
        ROS_ERROR("Error: Timed out.");
        break;
    case moveit::core::MoveItErrorCode::PREEMPTED:
        ROS_ERROR("Error: Preempted.");
        break;
    case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
        ROS_ERROR("Error: Start state in collision.");
        break;
    case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        ROS_ERROR("Error: Start state violates path constraints.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
        ROS_ERROR("Error: Goal in collision.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
        ROS_ERROR("Error: Goal violates path constraints.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
        ROS_ERROR("Error: Goal constraints violated.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
        ROS_ERROR("Error: Invalid group name.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        ROS_ERROR("Error: Invalid goal constraints.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
        ROS_ERROR("Error: Invalid robot state.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
        ROS_ERROR("Error: Invalid link name.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME:
        ROS_ERROR("Error: Invalid object name.");
        break;
    default:
        ROS_ERROR("Error: Unknown error.");
        break;
    }
    return false;
}

bool computeTrajectory(moveit::planning_interface::MoveGroupInterface &move_group,
                       const geometry_msgs::Pose &target_pose,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    // Lock the mutex if necessary
    mtx.lock();

    // Set the target pose directly
    move_group.setPoseTarget(target_pose);

    // Plan to the target pose
    bool success = handlePlanError(move_group.plan(plan), "planning");

    // Unlock the mutex if it was locked
    mtx.unlock();

    if (success)
    {
        ROS_INFO("Trajectory planning succeeded.");
    }
    else
    {
        ROS_WARN("Trajectory planning failed.");
    }

    return success;
}

bool moveInitialJoints(moveit::planning_interface::MoveGroupInterface& move_group, sensor_msgs::JointState joints)
{
    // Set the target joint values to match Unity's joint values
    move_group.setJointValueTarget(joints); // joint_positions should be a std::vector<double>

    // Execute the motion
    bool success = handlePlanError(move_group.move(), "executing");
    start_ready = true;
    return success;
}

void sendTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, ros::Publisher publish)
{
    moveit_msgs::RobotTrajectory& trajectory = plan.trajectory_;

    sensor_msgs::JointState joint_state_msg;
    
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = trajectory.joint_trajectory.joint_names;

    ros::Rate rate(60);

    for (const auto& point : trajectory.joint_trajectory.points){
        
        // / Add intereupt for if new pose is called later.
        // if (trajectroy_interupt)
        // {
        //     break;
        // }
        
        joint_state_msg.position = point.positions; // Joint positions for each joint
        joint_state_msg.velocity = point.velocities;
        joint_state_msg.effort = point.effort;
        // Publish the joint state
        publish.publish(joint_state_msg);

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;

    ros::Subscriber target_pose = node_handle.subscribe("weld_pose", 10, unityPoseCallback);
    ros::Subscriber unity_joints_sub = node_handle.subscribe("ur_joint_states", 10, unityJointStatesCallback);
    ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("unity_joint_commands", 10);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    

    // Set up the MoveIt! MoveGroup interface for the UR5e robot
    static const std::string PLANNING_GROUP = "manipulator"; // Replace with your robot's planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (ros::ok())
    {

        if (pose_received && joint_states_received)
        {

            if (start_ready == false){
                moveInitialJoints(move_group, unity_joints);
                ROS_INFO("Moving sim to match unity position.");
                ros::Duration(1.0).sleep(); // Wait before checking again
            }
            
            // Call the computeTrajectory function with the pose and joint states
            if (computeTrajectory(move_group, incoming_pose, my_plan))
            {
                sendTrajectory(my_plan, joint_pub);
                move_group.execute(my_plan);
                start_ready = false;
            }
            else
            {
                ROS_WARN("Unable to execute trajectory.");
            }

            // Reset flags after processing
            pose_received = false;
            joint_states_received = false;
        }
        else
        {
            ROS_WARN("Waiting for new poses and joint states...");
            ros::Duration(1.0).sleep(); // Wait before checking again
        }
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
