// using System.Collections;
// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.UnityRoboticsDemo;

// public class JointControl : MonoBehaviour
// {
//     public ArticulationBody joint; // Reference to this joint's ArticulationBody
//     public string jointName; // Name of the joint to subscribe to
//     private float targetPosition; // Target position for this joint

//     // void Start()
//     // {
//     //     // Create a ROS connection and subscribe to the joint commands
//     //     ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("ur_joint_states", UpdateJointTarget);
//     // }

//     void UpdateJointTarget(JointStateMsg jointState)
//     {
//         // Find the corresponding joint and set its target position from ROS data
//         for (int i = 0; i < jointState.name.Length; i++)
//         {
//             if (jointState.name[i] == jointName)
//             {
//                 targetPosition = (float)jointState.position[i] * Mathf.Rad2Deg; // Convert to degrees if needed
//                 break;
//             }
//         }
//     }

//     void FixedUpdate()
//     {
//         // Update the joint target position based on incoming ROS data
//         if (joint.jointType != ArticulationJointType.FixedJoint)
//         {
//             ArticulationDrive drive = joint.xDrive;
//             drive.target = targetPosition; // Set the joint's target position from ROS
//             joint.xDrive = drive;
//         }
//     }
// }

