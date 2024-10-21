#include <weldingrobot/weldingPath.h>

// Default Constructor
weldingPath::weldingPath() 
    : move_group_("manipulator") // default to manipulator if no group is provided
{
    move_group_.setMaxVelocityScalingFactor(0.1);
    move_group_.setMaxAccelerationScalingFactor(0.1);
}

// Constuctor with move_it tag
weldingPath::weldingPath(const std::string &planning_group)
    : move_group_(planning_group)
{
    move_group_.setMaxVelocityScalingFactor(0.1);
    move_group_.setMaxAccelerationScalingFactor(0.1);
}

// Method to add a pose to the vector
void weldingPath::addPose(const geometry_msgs::Pose &pose)
{
    poses_.push_back(pose);
    startPose_ = poses_.front();
    endPose_ = poses_.back();
}

// Method to get all poses
std::vector<geometry_msgs::Pose> weldingPath::getPoses() const
{
    return poses_;
}

// Method to get start pose
geometry_msgs::Pose weldingPath::getStart() const
{
    return startPose_;
}

// Method to get end pose
geometry_msgs::Pose weldingPath::getEnd() const
{
    return endPose_;
}

// Method to clear all poses
void weldingPath::clearPoses()
{
    poses_.clear();
}

// Method to return the number of poses stored
size_t weldingPath::poseCount() const
{
    return poses_.size();
}

bool weldingPath::computeTrajectory()
{
    if (poses_.empty())
    {
        ROS_WARN("No poses provided.");
        return false;
    }

    std::vector<geometry_msgs::Pose> waypoints = poses_;

    moveit_msgs::RobotTrajectory trajectory;

    // Create a MoveItErrorCodes object
    moveit_msgs::MoveItErrorCodes error_code;

    double eef_step = 0.01; // End effector step size
    double jump_threshold = 0.0; // Jump threshold
    bool avoid_collisions = true; // Collision avoidance

    // Compute the Cartesian path
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions, &error_code);

    switch (error_code.val) {
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        ROS_WARN("Path planning failed: PLANNING_FAILED");
        break;
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        ROS_WARN("Path planning failed: INVALID_MOTION_PLAN");
        break;
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        ROS_WARN("Path planning failed: NO_IK_SOLUTION (No valid IK solution found)");
        break;
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
        ROS_WARN("Path planning failed: INVALID_ROBOT_STATE (Invalid robot state for planning)");
        break;
    default:
        ROS_INFO("Path planning is successful, path achieved: %.2f.", fraction);
        break;
    }

    // Check the error code
    if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
       return false;
    }
    // Store the trajectory for later execution
    planned_trajectory_ = trajectory;
    return true;
}

// Execute the planned trajectory
void weldingPath::executeTrajectory()
{
    if (planned_trajectory_.joint_trajectory.points.empty())
    {
        ROS_WARN("No valid trajectory to execute.");
        return;
    }

    move_group_.execute(planned_trajectory_);
}

// Method for single pose trajectory
bool weldingPath::singlePoseTrajectory(geometry_msgs::Pose Pose)
{

    move_group_.setPoseTarget(Pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Check if path is possible
    if (success == false)
    {
        ROS_WARN("Planning failed.");
        return false;
    }

    ROS_INFO("Planning successful.");
    return true;
}

//Method for moving into starting position
bool weldingPath::startWeldPosition()
{
    if (singlePoseTrajectory(poses_.front()))
    {
        ROS_INFO("Moving to desired pose");
        move_group_.move();
        poses_.erase(poses_.begin());
        return true;
    }
    else{
        ROS_ERROR("Failure to move to start position");
        return false;
    }
}
