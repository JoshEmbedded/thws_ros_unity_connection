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
bool pose_recieved = false;
bool joints_received = false;

std::mutex mtx; // Create a mutex
int sharedCounter = 0; // Shared variable

void unityPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){

    ROS_INFO("NEW POSES RECEIVED...");
    ros::Duration(0.1).sleep(); // Sleep for 100 ms
    mtx.lock();
    incoming_pose = *msg;
    ROS_INFO("Sphere pose: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)", 
             incoming_pose.position.x, incoming_pose.position.y, incoming_pose.position.z, 
             incoming_pose.orientation.x, incoming_pose.orientation.y, incoming_pose.orientation.z, incoming_pose.orientation.w);
    pose_recieved = true;
    mtx.unlock();
}

void unityJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
    ROS_INFO("NEW JOINTS STATES RECEIVED...");
    ros::Duration(0.1).sleep(); // Sleep for 100 ms
    unity_joints = *msg;
    
    std::ostringstream joint_info;
    for (size_t i = 0; i < unity_joints.name.size(); ++i) {
        joint_info << "Joint Name: " << unity_joints.name[i] 
                   << ", Position: " << unity_joints.position[i]
                   << ", Velocity: " << unity_joints.velocity[i]
                   << ", Effort: " << unity_joints.effort[i] << "\n";
    }
    joints_received = true;
    
    ROS_INFO_STREAM("Unity Joint States:\n" << joint_info.str());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;

    ros::Subscriber target_pose = node_handle.subscribe("weld_pose", 10, unityPoseCallback);
    ros::Subscriber unity_joints = node_handle.subscribe("ur_joint_states", 10, unityJointStatesCallback);


    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {

        if (!pose_recieved)
        {
            ROS_WARN("No poses received. Waiting for new poses...");
            ros::Duration(1.0).sleep(); // Optionally, wait for a second before checking again
            continue;                   // Skip the rest of the loop and start again
        }

        if (!joints_received)
        {
            ROS_WARN("No joint_states received. Waiting for new joints...");
            ros::Duration(1.0).sleep(); // Optionally, wait for a second before checking again
            continue;                   // Skip the rest of the loop and start again
        }

    }

    spinner.stop();
    ros::shutdown();
    return 0;

}