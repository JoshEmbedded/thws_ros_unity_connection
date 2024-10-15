#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <weldingrobot/weldingPath.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_send");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Initalise object of welding path
    weldingPath robot("manipulator");

    // // Example poses to pass into the planner
    // std::vector<geometry_msgs::Pose> poses;

    // Pose 1
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.4;
    pose1.position.y = 0.2;
    pose1.position.z = 0.5;
    pose1.orientation.w = 1.0;
    robot.addPose(pose1);

    // Pose 2
    geometry_msgs::Pose pose2;
    pose2.position.x = 0.5;
    pose2.position.y = 0.3;
    pose2.position.z = 0.6;
    pose2.orientation.w = 1.0;
    robot.addPose(pose2);

    // move_group.setPoseTarget(pose1);

    // Compute and execute trajectory
    if (robot.computeTrajectory())
    {
        robot.executeTrajectory();
    }

    ros::shutdown();
    return 0;
}

