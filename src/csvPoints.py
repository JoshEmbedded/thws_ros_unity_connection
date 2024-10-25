#!/usr/bin/env python

import pandas as pd
import numpy as np
from pathlib import Path
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
import time
import portalocker
import tf.transformations as tf_trans

def updatePoints():
    # Read the CSV file
    base_dir = Path.home() / "weldingrobot"
    file_path = base_dir / "include" / "oculus" / "csv" / "Path_1_corrected.csv"

    # Briefly lock and read the file
    with open(file_path, 'r') as file:
        portalocker.lock(file, portalocker.LOCK_EX)
        df = pd.read_csv(file)  # Read CSV file contents
        portalocker.unlock(file)
        
    df_list = df.values.tolist()

    return df_list


def getWeldingCoordinates(df_list):
    
    pose_array = PoseArray()  # Initialize a PoseArray
    pose_array.poses = []  # Initialize the poses list within PoseArray

    for point in df_list:
        pose = Pose()  # Create a Pose object
        pose.position.x = point[1] * 10**-3  # X
        pose.position.y = point[2] * 10**-3  # Y 
        pose.position.z = point[3] * 10**-3  # Z 
        
        # rotation conversion from rot to quarternion
        
        quaternion = tf_trans.quaternion_from_euler(np.deg2rad(point[4]),np.deg2rad(point[5]),np.deg2rad(point[6]))
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pose_array.poses.append(pose)  # Add the pose to the poseArray

    return pose_array  # Return the populated poseArray


def pose_array_publisher():
    
    rospy.init_node('oculus_pose', anonymous=True)
    pub = rospy.Publisher('weld_pose_topic', PoseArray, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    last_update_time = time.time() - 10  # Ensure updatePoints runs immediately on first loop

    points = updatePoints()  # Initial load of points
    while not rospy.is_shutdown():
        # Update points every 10 seconds
        if time.time() - last_update_time >= 10:
            points = updatePoints()  # Refresh points from CSV
            last_update_time = time.time()

        pose_array = getWeldingCoordinates(points)  # Get PoseArray
        pub.publish(pose_array)  # Publish the PoseArray
        rate.sleep()  # Sleep to maintain the publish rate

if __name__ == '__main__':
    try:
        pose_array_publisher()
    except rospy.ROSInterruptException:
        pass