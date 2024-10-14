#ifndef WELDING_PATH_H
#define WELDING_PATH_H

#include <vector>
#include <geometry_msgs/Pose.h>

class weldingPath {
private:
    std::vector<geometry_msgs::Pose> poses;  // Vector to store poses
    geometry_msgs::Pose startPose; // Pose for starting weld
    geometry_msgs::Pose endPose; // Pose for ending weld

public:

    weldingPath();

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
};

#endif WELDING_PATH_H