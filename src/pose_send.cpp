#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <weldingrobot/weldingPath.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/DisplayTrajectory.h>

std::vector<geometry_msgs::Pose> received_poses; // Global or class member to store received poses

void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    // Clear previous poses
    received_poses.clear();

    // Store the received poses
    received_poses = msg->poses;

    // Log the received poses
    for (const auto &pose : received_poses)
    {
        ROS_INFO("Received Pose - x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_send");
    ros::NodeHandle node_handle;

    // Initalise object of welding path
    weldingPath robot("manipulator");

    ros::Subscriber welding_pose_sub = node_handle.subscribe("weld_pose_topic", 10, poseArrayCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Keep running the node until shutdown
    while (ros::ok())
    {

        // Check if you have valid poses before processing
        if (robot.poseCount() == 0)
        {
            ROS_WARN("No poses received. Waiting for new poses...");
            robot.addPoses(received_poses);
            ros::Duration(1.0).sleep(); // Optionally, wait for a second before checking again
            continue;                   // Skip the rest of the loop and start again
        }

        // Send robot to starting postion
        if (!(robot.startWeldPosition()))
        {
            ros::shutdown();
            return 0;
        }

        // Pause and wait for user input
        if (robot.computeTrajectory())
        {
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

        // You might want to add a small sleep to avoid busy-waiting
        ros::Duration(0.1).sleep(); // Sleep for 100 ms
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
