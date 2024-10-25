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
#include <cmath> // For std::fabs()

std::vector<geometry_msgs::Pose> received_poses; // Global or class member to store received poses
bool new_poses_flag = false;                     // flag for new pose

// Function to compare two Pose objects with a tolerance
bool are_poses_approx_equal(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double tolerance)
{
    // Compare position
    if (std::fabs(p1.position.x - p2.position.x) > tolerance ||
        std::fabs(p1.position.y - p2.position.y) > tolerance ||
        std::fabs(p1.position.z - p2.position.z) > tolerance)
    {
        return false;
    }

    // Compare orientation
    if (std::fabs(p1.orientation.x - p2.orientation.x) > tolerance ||
        std::fabs(p1.orientation.y - p2.orientation.y) > tolerance ||
        std::fabs(p1.orientation.z - p2.orientation.z) > tolerance ||
        std::fabs(p1.orientation.w - p2.orientation.w) > tolerance)
    {
        return false;
    }

    return true;
}

// Function to check if two vectors of Pose are approximately equal
bool are_poses_equal(const std::vector<geometry_msgs::Pose> &poses1, const std::vector<geometry_msgs::Pose> &poses2, double tolerance)
{
    if (poses1.size() != poses2.size())
    {
        return false;
    }

    // Assume they are identical unless we find a difference
    bool identical_check = true;

    for (size_t i = 0; i < poses1.size(); ++i)
    {
        if (!are_poses_approx_equal(poses1[i], poses2[i], tolerance))
        {
            identical_check = false; // Foound a difference
            break;
        }
    }

    return identical_check;
}

void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    // Set a small tolerance value (e.g., 0.001 for 1 mm/1e-3 m tolerance)
    double tolerance = 0.001; // Adjust this value based on your precision needs

    // Check if the new poses are the same as the currently stored poses within the tolerance
    if (received_poses.empty() || !are_poses_equal(received_poses, msg->poses, tolerance))
    {
        
        ROS_INFO("NEW POSES RECIEVED...");
        ros::Duration(0.1).sleep(); // Sleep for 100 ms
        // New poses are different or we haven't received any poses yet
        received_poses = msg->poses;
        new_poses_flag = true; // new poses in system
        // Log the received poses
        for (const auto &pose : received_poses)
        {
            ROS_INFO("Received Pose - x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z);
        }
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
        if (new_poses_flag == false)
        {
            ROS_WARN("No poses received. Waiting for new poses...");
            ros::Duration(1.0).sleep(); // Optionally, wait for a second before checking again
            continue;                   // Skip the rest of the loop and start again
        }

        if (new_poses_flag == true)
        {
            
            robot.addPoses(received_poses);

            // Send robot to starting postion
            if (!(robot.startWeldPosition()))
            {
                ros::shutdown();
                return 0;
            }

            ROS_INFO("Robot at starting postion...");

            // Pause and wait for user input
            if (robot.computeTrajectory())
            {
                std::string user_input;
                std::cout << "Do you want to execute the trajectory? (y/n): ";
                std::cin >> user_input;

                if (user_input == "y" || user_input == "Y")
                {
                    robot.executeTrajectory();
                    new_poses_flag = false;
                }
                else
                {
                    ROS_INFO("Trajectory execution cancelled.");
                }
            }
        }

        // You might want to add a small sleep to avoid busy-waiting
        ros::Duration(0.1).sleep(); // Sleep for 100 ms
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
