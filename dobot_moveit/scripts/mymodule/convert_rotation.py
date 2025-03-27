#!/usr/bin/env python3
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose

def eulertoquat(euler_angles):
    rot = Rotation.from_euler("xyz", euler_angles, degrees=True)
    qua = rot.as_quat()
    msg = Pose()
    msg.position.x = msg.position.y = msg.position.z = 0.0
    msg.orientation.x = qua[0]
    msg.orientation.y = qua[1]
    msg.orientation.z = qua[2]
    msg.orientation.w = qua[3]
    return msg.orientation
