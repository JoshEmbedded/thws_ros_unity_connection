
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor; // For JointState message type
using RosMessageTypes.Std;    // For HeaderMsg
using RosMessageTypes.BuiltinInterfaces;

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
        Vector3 position = sphere.transform.position;
        Quaternion rotation = sphere.transform.rotation;

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
        // Ensure jointPosition has at least one element
        if (joints[i].jointPosition.dofCount > 0)
        {
            jointPositions[i] = joints[i].jointPosition[0]; // Assuming single DOF joints
        }
        else
        {
            Debug.LogWarning("Joint " + joints[i].name + " does not have a position.");
            jointPositions[i] = 0; // Set to a default value or handle appropriately
        }
    }

    // Create a header for the joint state message
    var header = new HeaderMsg
    {
        stamp = new TimeMsg((uint)Time.time, (uint)((Time.time - (uint)Time.time) * 1e9)), // Correct way to create timestamp
        frame_id = "ur_robot" // Use a descriptive frame_id for the robot
    };

    // Create and publish the joint state message
    JointStateMsg jointStateMsg = new JointStateMsg(header, jointNames, jointPositions, new double[joints.Length], new double[joints.Length]);
    ros.Publish(jointStateTopicName, jointStateMsg);
    }

}
