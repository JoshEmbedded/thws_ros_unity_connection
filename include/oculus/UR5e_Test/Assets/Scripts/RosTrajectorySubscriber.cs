using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using System.Linq;


public class RosTrajectorySubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/unity_joint_commands";
    const int k_NumRobotJoints = 6;

    // Hardcoded variables
    const float k_JointAssignmentWait = 0.038f;
    

    // Public GameObject variable to hold the robot
    public GameObject robot;

    private ArticulationBody[] robotArticulationBody; // Private variable to hold the ArticulationBody reference

    private Dictionary<string, string> jointToLinkMapping = new Dictionary<string, string>
    {
        { "shoulder_pan_joint", "shoulder_link" },
        { "shoulder_lift_joint", "upper_arm_link" },
        { "elbow_joint", "forearm_link" },
        { "wrist_1_joint", "wrist_1_link" },
        { "wrist_2_joint", "wrist_2_link" },
        { "wrist_3_joint", "wrist_3_link" }
    };

    public static readonly string[] LinkNames =
        { "base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        robotArticulationBody = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
    robotArticulationBody[i] = robot.transform.Find(linkName)?.GetComponent<ArticulationBody>();
            if (robotArticulationBody[i] == null)
    {
        Debug.LogError($"Could not find ArticulationBody component for {linkName}");
    }
        }
    }

    void Start()
    {
        // Initialize the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        Debug.Log("Attempting to connect to ROS...");
        ros.Subscribe<JointStateMsg>(topicName, ExecuteTrajectories);
        Debug.Log($"Subscribed to topic: {topicName}");

        // Awake();
        // ArticulationBody[] robotArticulationBody = robot.GetComponentsInChildren<ArticulationBody>();

        // if (robotArticulationBody == null)
        // {
        //     Debug.LogError("ArticulationBody component not found on the assigned GameObject!");
        // }
        // Print out all ArticulationBody components in the robot
        // PrintArticulationBodies();
    }

    // Print all ArticulationBody components in the robot GameObject
    // private void PrintArticulationBodies()
    // {
    //     // ArticulationBody articulationBodies = robot.GetComponentsInChildren<ArticulationBody>();
    //     foreach (ArticulationBody body in robotArticulationBody)
    //     {
    //         Debug.Log($"ArticulationBody found: {body.name}");
    //     }
    // }

    

    // private void OnJointStateReceived(JointStateMsg jointState)
    // {
    //     Debug.Log("Received joint state!");

    //     Debug.Log($"Received {jointState.position.Length} positions from ROS.");
    //     Debug.Log($"Articulation body has {robot.dofCount} DOF.");

    //     // Create a float array to hold the positions
    //     List<float> positions = new List<float>();

    //     // Convert each double position to float and add to the positions list
    //     for (int i = 0; i < jointState.position.Length; i++)
    //     {
    //         float position = (float)jointState.position[i]; // Convert double to float
    //         positions.Add(position); // Add to the list
    //         Debug.Log($"Joint: {jointState.name[i]}, Position: {position}"); // Log each joint position
    //     }

    //     robot.SetJointPositions(positions);

    //     // robotArticulationBody.SetJointPositions(positions);

    // }

    /// <summary>
    ///     Execute trajectories from RobotMoveActionGoal topic.
    ///     Execution will iterate through every robot pose in the trajectory pose
    ///     array while updating the joint values on the simulated robot.
    /// </summary>
    /// <param name="trajectories"> The array of poses for the robot to execute</param>
    private void ExecuteTrajectories(JointStateMsg jointState)
    {
            
            var jointPositions = jointState.position;
            var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            // Set the joint values for every joint
            for (var joint = 0; joint < robotArticulationBody.Length; joint++)
            {
                var joint1XDrive = robotArticulationBody[joint].xDrive;
                joint1XDrive.target = result[joint];
                robotArticulationBody[joint].xDrive = joint1XDrive;
            }

    }
    

    
    
    // // Callback method for receiving joint state messages
    // private void OnJointStateReceived(JointStateMsg jointState)
    // {
    //     Debug.Log("Received joint state!");

    //     if (jointState.name.Length != jointState.position.Length)
    //     {
    //         Debug.LogError("Mismatch between joint names and positions array lengths!");
    //         return; // Early exit to prevent out-of-bounds access
    //     }

    //     for (int i = 0; i < jointState.name.Length; i++)
    //     {
    //         string jointName = jointState.name[i];
    //         float jointPosition = (float)jointState.position[i]; // Explicitly cast to float
            
    //         // Log joint name and position
    //         Debug.Log($"Joint: {jointName}, Position: {jointPosition}");

    //         // Find the corresponding link in the robot GameObject
    //         if (jointToLinkMapping.TryGetValue(jointName, out string linkName))
    //         {
    //             // Find the ArticulationBody component for the link
    //             ArticulationBody articulationBody = robot.transform.Find(linkName)?.GetComponent<ArticulationBody>();
    //             if (articulationBody != null)
    //             {
    //                 // Set the target position of the articulation drive
    //                 ArticulationDrive drive = articulationBody.xDrive; // Use xDrive, yDrive, or zDrive as necessary
    //                 drive.target = jointPosition * Mathf.Rad2Deg; // Convert radians to degrees if your joint is defined in radians
    //                 articulationBody.xDrive = drive; // Reassign the modified drive back to the articulation body
    //             }
    //             else
    //             {
    //                 Debug.LogWarning($"ArticulationBody for link {linkName} not found.");
    //             }
    //         }
    //         else
    //         {
    //             Debug.LogWarning($"Joint {jointName} not found in mapping.");
    //         }
    //     }
    // }



    
}
