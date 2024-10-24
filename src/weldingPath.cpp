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

void weldingPath::addPoses(std::vector<geometry_msgs::Pose> pose_array)
{
    poses_.insert(poses_.end(), pose_array.begin(), pose_array.end());
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
        ROS_WARN("No poses set for execution.");
        return false;
    }

    std::vector<geometry_msgs::Pose> waypoints = poses_;

    moveit_msgs::RobotTrajectory trajectory;

    // Create a MoveItErrorCodes object
    moveit_msgs::MoveItErrorCodes error_code;

    double eef_step = 0.01;       // End effector step size
    double jump_threshold = 0.0;  // Jump threshold
    bool avoid_collisions = true; // Collision avoidance

    // Compute the Cartesian path
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions, &error_code);

    if (!(handlePlanError(error_code))){
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

// bool weldingPath::weldPath(){

// } 

// Method for moving into starting position
bool weldingPath::startWeldPosition()
{
    move_group_.setPoseTarget(poses_.front());

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    if (handlePlanError(move_group_.plan(my_plan)))
    {
        // Plan was successful, proceed with execution
        moveit::core::MoveItErrorCode execution_result = move_group_.execute(my_plan);

        if (handlePlanError(execution_result))
        {
            ROS_INFO("Moving to desired pose");
            return true;
        }

        else
        {
            ROS_ERROR("Failure to move to start position");
            return false;
        }
    }
    return false;
}

// Method for checking MoveIt error
bool weldingPath::handlePlanError(moveit::core::MoveItErrorCode my_plan)
{
    // Handle different error codes explicitly
    switch (my_plan.val)
    {
    case moveit::core::MoveItErrorCode::SUCCESS:
        ROS_INFO("Planning successful!");
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
