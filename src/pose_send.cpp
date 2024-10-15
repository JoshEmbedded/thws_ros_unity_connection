#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <weldingrobot/weldingPath.h>
#include <iostream> // Include for std::cin

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_send");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Initalise object of welding path
    weldingPath robot("manipulator");

    // Create a publisher to publish planned trajectories to RViz
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);

    // Set the publisher in the weldingPath object
    robot.setDisplayPublisher(display_publisher);

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
        // Ask for user confirmation to execute the trajectory
        std::string user_input;
        std::cout << "Do you want to execute the trajectory? (y/n): ";
        std::cin >> user_input;

        if (user_input == "y" || user_input == "Y")
        {
            robot.executeTrajectory();
        }
        else
        {
            std::cout << "Trajectory execution canceled." << std::endl;
        }
    }

    ros::shutdown();
    return 0;
}

