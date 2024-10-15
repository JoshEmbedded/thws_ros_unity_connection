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

    double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction < 0.95)
    {
        ROS_WARN("Could not plan a complete trajectory, only %.2f of the path was planned.", fraction);
        return false;
    }

    // Store the trajectory for later execution
    planned_trajectory_ = trajectory;

    // Visualize the planned trajectory in RViz
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Use getCurrentState() directly to populate the RobotState
    moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
    moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start);

    // Publish the trajectory to RViz
    display_publisher_.publish(display_trajectory);

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

void weldingPath::setDisplayPublisher(const ros::Publisher &publisher)
{
    display_publisher_ = publisher;
}
