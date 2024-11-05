
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor; // For JointState message type
using RosMessageTypes.Std;    // For HeaderMsg
using RosMessageTypes.BuiltinInterfaces;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
///
/// </summary>
public class CustomRosPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string weldPoseTopicName = "weld_pose"; // Topic for the sphere's pose
    public string jointStateTopicName = "ur_joint_states"; // Topic for joint states

    // Public variable to specify the robot GameObject
    public GameObject robot; // Drag and drop the robot GameObject in the Inspector
    
    // The GameObject for the sphere
    public GameObject sphere; 
    // Publish frequency
    public float publishMessageFrequency = 0.5f;

    // For publishing joint states
    private ArticulationBody[] joints;
    private string[] jointNames;
    private double[] jointPositions; // Using double for ROS compatibility

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(weldPoseTopicName); // Register pose publisher
        ros.RegisterPublisher<JointStateMsg>(jointStateTopicName); // Register joint state publisher

        // Get all ArticulationBody components from the specified robot GameObject
        joints = robot.GetComponentsInChildren<ArticulationBody>();

        // Initialize names and positions arrays
        jointNames = new string[joints.Length];
        jointPositions = new double[joints.Length];

        // Set joint names based on the names of the ArticulationBodies
        for (int i = 0; i < joints.Length; i++)
        {
            jointNames[i] = joints[i].name; // Adjust naming scheme if necessary
        }
        Debug.Log("Number of joints found: " + joints.Length);

    }

    public void UpdatePoseAndPublishJointStates()
    {
        UpdatePose();          // Call function to update pose
        PublishJointStates();  // Call function to publish joint states
    }

    private void UpdatePose()
    {
        // Get the current position and rotation of the sphere
        Vector3<FLU> position = sphere.transform.position.To<FLU>();
        Quaternion<FLU> rotation = sphere.transform.rotation.To<FLU>();

        // Create the pose message
        PoseMsg spherePos = new PoseMsg(
            new PointMsg(position.x, position.y, position.z),
            new QuaternionMsg(rotation.x, rotation.y, rotation.z, rotation.w)
        );

        // Publish the pose of the sphere
        ros.Publish(weldPoseTopicName, spherePos); 
    }

    private void PublishJointStates()
    {
    // Update joint positions
    for (int i = 0; i < joints.Length; i++)
    {
        string moveitJointName;

        try 
        {
            jointPositions[i] = joints[i].jointPosition[0];
            Debug.Log($"joint: {joints[i].jointPosition[0]}");
            
            if (linkToJointMapping.TryGetValue(joints[i].name, out moveitJointName))
            {
                jointNames[i] = moveitJointName; // Use MoveIt joint name
            }
            else
            {
                Debug.LogWarning("Link name " + joints[i].name + " not found in mapping dictionary.");
            }
        }
        
        catch
        {
            Debug.Log($"Information failed being stored in Joint: {joints[i].name}, Waa Waa it didnt work");
        }

        
    }

    // Create a header for the joint state message
    var header = new HeaderMsg
    {
        stamp = new TimeMsg((uint)Time.time, (uint)((Time.time - (uint)Time.time) * 1e9)), // Correct way to create timestamp
        frame_id = "ur_robot" // Use a descriptive frame_id for the robot
    };

    // Create and publish the joint state message
    JointStateMsg jointStateMsg = new JointStateMsg(header, jointNames, jointPositions, new double[jointNames.Length], new double[jointNames.Length]);
    ros.Publish(jointStateTopicName, jointStateMsg);
    }

    private Dictionary<string, string> linkToJointMapping = new Dictionary<string, string>
    {
        { "shoulder_link", "shoulder_pan_joint" },
        { "upper_arm_link", "shoulder_lift_joint" },
        { "forearm_link", "elbow_joint" },
        { "wrist_1_link", "wrist_1_joint" },
        { "wrist_2_link", "wrist_2_joint" },
        { "wrist_3_link", "wrist_3_joint" },
        { "flange", "flange"},
        { "tool0", "tool0"}
    };

}
