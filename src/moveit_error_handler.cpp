#include "weldingrobot/moveit_error_handler.h"
#include <moveit_msgs/MoveItErrorCodes.h>

namespace moveit_error_handler
{
    bool handlePlanError(moveit::core::MoveItErrorCode my_plan, std::string planning)
    {
        switch (my_plan.val)
        {
        case moveit::core::MoveItErrorCode::SUCCESS:
            if (planning == "planning"){
                ROS_INFO("Planning successful!");
            }
            else{
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
}
