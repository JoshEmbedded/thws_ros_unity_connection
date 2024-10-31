#ifndef MOVEIT_ERROR_HANDLER_H
#define MOVEIT_ERROR_HANDLER_H

#include <moveit_msgs/MoveItErrorCodes.h>
#include <string>
#include <ros/ros.h>

namespace moveit_error_handler
{
    // Function to handle MoveIt error codes and provide feedback
    bool handlePlanError(moveit::core::MoveItErrorCode my_plan, std::string planning);
}

#endif // MOVEIT_ERROR_HANDLER_H
