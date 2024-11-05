using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using System.Linq;

public class RosJointController : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/unity_joint_commands";

    // Public GameObject variable to hold the robot
    public GameObject robot;

    private ArticulationBody[] robotArticulationBody; // Private variable to hold the ArticulationBody reference

    void Start()
    {
        // Initialize the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        Debug.Log("Attempting to connect to ROS...");
        ros.Subscribe<JointStateMsg>(topicName, ExecuteTrajectories);
        Debug.Log($"Subscribed to topic: {topicName}");
        robotArticulationBody = robot.GetComponentsInChildren<ArticulationBody>();

        if (robotArticulationBody == null || robotArticulationBody.Length == 0)
        {
            Debug.LogError("robotArticulationBody array is not initialized.");
            Debug.LogError($"number of joints: {robotArticulationBody.Length}");
        }

    }

    private void ExecuteTrajectories(JointStateMsg jointState)
    {
        Debug.Log("Received joint state!");
        
        List<float> positions = jointState.position
        .Select(d => (float)d * Mathf.Rad2Deg) // Convert each element to float and from radians to degrees
        .ToList();

        int counter = 0;

        // Set the joint values for every joint
        for (var joint = 0; joint < robotArticulationBody.Length; joint++)
        {
                Debug.Log($"Counter: {counter++}");
                var joint1XDrive = robotArticulationBody[joint].xDrive;
                joint1XDrive.target = positions[joint];
                robotArticulationBody[joint].xDrive = joint1XDrive;
            
        }

    }
}