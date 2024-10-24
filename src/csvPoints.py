#!/usr/bin/env python

import pandas as pd
import numpy as np
from pathlib import Path
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R



def updatePoints():
    # Read the CSV file
    base_dir = Path.home() / "weldingrobot"
    file_path = base_dir / "include" / "oculus" / "csv" / "Path_1_corrected.csv"

    df = pd.read_csv(file_path)
    df_list = df.values.tolist()

    # Extract the rotation angles (assuming they're in degrees)
    rot_x = df['RotX'].values
    rot_y = df['RotY'].values
    rot_z = df['RotZ'].values

    # Specify the rotation order
    rotation_order = 'xyz'

    # Create a Rotation object from Euler angles
    rotations = R.from_euler(rotation_order, np.column_stack((rot_x, rot_y, rot_z)), degrees=True)

    # Convert to quaternions
    quaternions = rotations.as_quat()  # This gives you an array of quaternions

    # Add quaternion values to the original list
    # Assuming df_list originally contains [X, Y, Z] format
    for idx, quaternion in enumerate(quaternions):
        df_list[idx].extend(quaternion.tolist())  # Append quaternion to each list entry
        
    return df_list


def getWeldingCoordinates(df_list):
    
    pose_array = PoseArray()  # Initialize a PoseArray
    pose_array.poses = []  # Initialize the poses list within PoseArray

    for point in df_list:
        pose = Pose()  # Create a Pose object
        pose.position.x = point[1] * 10**-3  # X
        pose.position.y = point[2] * 10**-3  # Y 
        pose.position.z = point[3] * 10**-3  # Z 
        pose.orientation.x = point[7]  # Quaternion X
        pose.orientation.y = point[8]  # Quaternion Y
        pose.orientation.z = point[9]  # Quaternion Z
        pose.orientation.w = point[10]  # Quaternion W

        pose_array.poses.append(pose)  # Add the pose to the poseArray

    return pose_array  # Return the populated poseArray


def pose_array_publisher():
    
    rospy.init_node('oculus_pose', anonymous=True)
    pub = rospy.Publisher('weld_pose_topic', PoseArray, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        points = updatePoints()  # Update points from CSV
        pose_array = getWeldingCoordinates(points)  # Get PoseArray
        pub.publish(pose_array)  # Publish the PoseArray
        rate.sleep()  # Sleep to maintain the publish rate

if __name__ == '__main__':
    try:
        pose_array_publisher()
    except rospy.ROSInterruptException:
        pass