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
            Debug.Log($"Searching for link: {linkName}");
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
    }

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

                if (robotArticulationBody == null) {
                Debug.LogError("robotArticulationBody array is null!");
                return;
            }

            if (joint < 0 || joint >= robotArticulationBody.Length) {
                Debug.LogError($"Invalid joint index: {joint}. Array length: {robotArticulationBody.Length}");
                return;
            }

            if (robotArticulationBody[joint] == null) {
                Debug.LogError($"ArticulationBody at index {joint} is null!");
                return;
            }
                var joint1XDrive = robotArticulationBody[joint].xDrive;
                joint1XDrive.target = result[joint];
                robotArticulationBody[joint].xDrive = joint1XDrive;
            }

    }
}
