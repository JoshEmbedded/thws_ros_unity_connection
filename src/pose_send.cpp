#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <weldingrobot/weldingPath.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/DisplayTrajectory.h>

/*
/// This is an example callback to use later for the oculus welding points
void weldingPoseCallback(const std::vector<geometry_msgs::Pose msg)
{

}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_send");
    ros::NodeHandle node_handle;

    /*
    // Prepare a fake subscriber for when the points are published from Oculus

    ros::Subscriber sub = node_handle.subscribe("<topic_name for pose>", 1000, weldingPoseCallback);

    */

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initalise object of welding path
    weldingPath robot("manipulator");

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

    // Pose 3
    geometry_msgs::Pose pose3;
    pose3.position.x = -0.4;
    pose3.position.y = 0.4;
    pose3.position.z = 0.6;
    pose3.orientation.w = 1.0;
    robot.addPose(pose3);

    // Pose 2
    geometry_msgs::Pose pose4;
    pose4.position.x = -0.4;
    pose4.position.y = -0.4;
    pose4.position.z = 0.6;
    pose4.orientation.w = 1.0;
    robot.addPose(pose4);


    // Send robot to starting postion
    if (!(robot.startWeldPosition())){
        ros::shutdown();
        return 0;
    }

    // Pause and wait for user input
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
