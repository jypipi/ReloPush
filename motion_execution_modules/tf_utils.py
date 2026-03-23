#!/usr/bin/env python
"""
Utility module for common functions.
"""

import math
from geometry_msgs.msg import Quaternion

def angle_to_rosquaternion(yaw):
    """
    Convert a yaw angle (in radians) into a ROS Quaternion message.
    Assumes zero roll and zero pitch. The resulting quaternion represents
    a rotation about the Z axis by the angle 'yaw'.
    
    Parameters:
        yaw (float): The yaw angle in radians.
        
    Returns:
        Quaternion: A ROS geometry_msgs.msg.Quaternion representing the rotation.
    """
    quat = Quaternion()
    quat.w = math.cos(yaw / 2.0)
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    return quat

if __name__ == "__main__":
    # Quick test for the angle_to_rosquaternion function.
    test_yaw = math.pi / 4.0  # 45 degrees in radians
    q = angle_to_rosquaternion(test_yaw)
    print("Quaternion for yaw = 45° (in radians):")
    print("x: {:.3f}, y: {:.3f}, z: {:.3f}, w: {:.3f}".format(q.x, q.y, q.z, q.w))

