#!/usr/bin/env python3

import rospy
import actionlib
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def move_by_jointangles(msg):
    client = actionlib.SimpleActionClient("/me6_robot/joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    pub_msg = FollowJointTrajectoryGoal()
    pub_msg.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    point = JointTrajectoryPoint()
    point.positions = msg.position
    point.time_from_start = rospy.Duration(2)
    pub_msg.trajectory.points.append(point)
    client.send_goal(pub_msg)
    client.wait_for_result()
    result = client.get_result()
    if result:
        rospy.loginfo("Result:%s", result)

def reset_orientation(msg, move_group):
    client = actionlib.SimpleActionClient("/me6_robot/joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    client.cancel_goal()
    current_pose = move_group.get_current_pose().pose
    goal_pose = Pose()
    goal_pose.position = current_pose.position
    goal_pose.orientation.x = 0.5
    goal_pose.orientation.y = -0.5
    goal_pose.orientation.z = -0.5
    goal_pose.orientation.w = 0.5
    flag, plan, time, error = move_group.plan()
    if flag:
        goal_joints = plan.joint_trajectory.points[-1].positions
        current_joint_angles = move_group.get_current_joint_values()
        dif = goal_joints[0] - current_joint_angles[0]
        if dif > 0.5 or dif < -0.5:
            print("!-----out of range-----!")
            return None
        pub_msg = FollowJointTrajectoryGoal()
        pub_msg.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = goal_joints
        point.time_from_start = rospy.Duration(0.5)
        pub_msg.trajectory.points.append(point)
        rospy.loginfo(pub_msg.trajectory.points[0].positions)
        client.send_goal(pub_msg)
    else:                                                                                           
        print("!-----planning failed-----!")
    new_pose = move_group.get_current_pose().pose                                                   
    rospy.loginfo(goal_pose)                                                                        
    rospy.loginfo(new_pose)
