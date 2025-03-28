#!/usr/bin/env python3

import rospy
import moveit_commander
import actionlib
import threading
import sys
import argparse
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from mymodule import operation_pose_quaternion as op
from mymodule import set_constraints as sc
from mymodule import move_arm as ma

planning_thread = None
planning_lock = threading.Lock()
stop_flag = False
zero_pose = Pose()
zero_pose.position.x = 0
zero_pose.position.y = 0
zero_pose.position.z = 0
zero_pose.orientation.x = 0
zero_pose.orientation.y = 0
zero_pose.orientation.z = 0
zero_pose.orientation.w = 1

def goal_callback(state, result):
    rospy.loginfo("goal completed with state: %s, result: %s", state, result)
    
def planning(msg):
    global planning_thread, stop_flag
    # クライアントの生成
    client = actionlib.SimpleActionClient("/me6_robot/joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    client.cancel_goal() # 前のゴールのキャンセル
    rospy.loginfo("goal cancelled")
    if stop_flag:
        rospy.loginfo("thread interrupted")
        return None
    
    current_pose = move_group.get_current_pose().pose # 現在位置取得
    goal_pose = op.add_pose(current_pose, msg) # 現在位置と移動指令から目標位置算出
    move_group.set_pose_target(goal_pose) # 目標位置設定
    move_group.set_goal_orientation_tolerance(0.05) # エンドエフェクタ姿勢の許容誤差
    joint_constraint = sc.make_joint_constraint(move_group, 1.0) # 関節角の制限を設定
    move_group.set_path_constraints(joint_constraint) # 関節角の制限を適用
    rospy.loginfo("joint_constraints applied: %s", joint_constraint)
    move_group.set_planning_time(0.5) # タイムアウトの設定
    rospy.loginfo("plan start")
    flag, plan, time, error = move_group.plan() # 計画
    
    if flag:
        # 目標関節角を取得
        goal_joints = plan.joint_trajectory.points[-1].positions
        current_joint_angles = move_group.get_current_joint_values()
        dif = goal_joints[0] - current_joint_angles[0]
        rospy.loginfo("angle change of joint1 %s", dif)
        if dif > 0.5 or dif < -0.5:
            print("!-----out of range-----!")
            rospy.loginfo(goal_joints)
            return None
        # print(goal_joints)

        # ゴールの生成
        pub_msg = FollowJointTrajectoryGoal()
        pub_msg.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = goal_joints
        point.time_from_start = rospy.Duration(0.5)
        pub_msg.trajectory.points.append(point)
        rospy.loginfo("goal :%s", pub_msg.trajectory.points[0].positions)

        client.send_goal(pub_msg, done_cb=goal_callback) # ゴールを送信
        
    else:
        print("!-----planning failed-----!")
    # 現在位置の表示
    new_pose = move_group.get_current_pose().pose
    rospy.loginfo(goal_pose)
    rospy.loginfo(new_pose)

def callback(msg):
    global planning_thread, planning_lock, stop_flag, zero_pose
    if msg == zero_pose:
        return None
    with planning_lock:
        if planning_thread and planning_thread.is_alive():
            stop_flag = True
            planning_thread.join()
            stop_flag = False
        planning_thread = threading.Thread(target=planning, args=(msg,))
        planning_thread.start()
    
if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-t", "--wait_time", type=float, default=10.0)
    # time = parser.parse_args().wait_time
    rospy.init_node("test_node", anonymous=True)
    time = float(rospy.get_param("~wait_time", 0.0))
    rospy.sleep(time)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("me6_arm")

    rospy.Subscriber("magician_move", Pose, callback)
    rospy.Subscriber("magician_move_jointangles", JointState, ma.move_by_jointangles)
    rospy.Subscriber("magician_move_reset_orientation", Empty, ma.reset_orientation, callback_args=move_group)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
