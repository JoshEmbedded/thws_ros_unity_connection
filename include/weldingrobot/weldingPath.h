#ifndef WELDING_PATH_H
#define WELDING_PATH_H


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <geometry_msgs/Pose.h>

class weldingPath {
private:
    std::vector<geometry_msgs::Pose> poses_;  // Vector to store poses
    geometry_msgs::Pose startPose_; // Pose for starting weld
    geometry_msgs::Pose endPose_; // Pose for ending weld

    
    moveit_msgs::RobotTrajectory planned_trajectory_;

public:

    moveit::planning_interface::MoveGroupInterface move_group_;

    weldingPath();

    weldingPath(const std::string &planning_group);

    // Method to add a pose to the vector
    void addPose(const geometry_msgs::Pose& pose);

    // Method to get all poses
    std::vector<geometry_msgs::Pose> getPoses() const;

    // Method to get start pose
    geometry_msgs::Pose getStart() const;

    // Method to get end pose
    geometry_msgs::Pose getEnd() const;

    // Optional: method to clear all poses
    void clearPoses();
    
    // Optional: method to return the number of poses stored
    size_t poseCount() const;

    // Function to set poses and compute trajectory
    bool computeTrajectory();

    // Execute the planned trajectory
    void executeTrajectory();

    // Evaluate single pose trajectory
    bool singlePoseTrajectory(geometry_msgs::Pose Pose);

    // Method for moving into starting position
    bool startWeldPosition();

};

#endif // WELDING_PATH_H