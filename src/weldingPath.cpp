#include "weldingPath.h"

// Constructor
weldingPath::weldingPath() {}

// Method to add a pose to the vector
void weldingPath::addPose(const geometry_msgs::Pose& pose) {
    poses.push_back(pose);
    startPose = poses.front();
    endPose = poses.back();
}

// Method to get all poses
std::vector<geometry_msgs::Pose> weldingPath::getPoses() const {
    return poses;
}

// Method to get start pose
geometry_msgs::Pose weldingPath::getStart() const{
    return startPose;
}

// Method to get end pose    
geometry_msgs::Pose weldingPath::getEnd() const{
    return endPose;
} 

// Method to clear all poses
void weldingPath::clearPoses() {
    poses.clear();
}

// Method to return the number of poses stored
size_t weldingPath::poseCount() const {
    return poses.size();
}
