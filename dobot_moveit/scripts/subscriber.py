#!/usr/bin/env python3

import rospy
import moveit_commander
import actionlib
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
from std_msgs.msg import Empty
from mymodule import operation_pose_quaternion as op
from mymodule import set_constraints as sc
from mymodule import move_arm as ma

def goal_callback(state, result):
    rospy.loginfo("goal completed with state: %s, result: %s", state, result)
    
def callback(msg):
    # クライアントの生成
    client = actionlib.SimpleActionClient("/me6_robot/joint_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    client.cancel_goal() # 前のゴールのキャンセル
    rospy.loginfo("goal cancelled")
        
    current_pose = move_group.get_current_pose().pose # 現在位置取得
    goal_pose = op.add_pose(current_pose, msg) # 現在位置と移動指令から目標位置算出
    move_group.set_pose_target(goal_pose) # 目標位置設定
    move_group.set_goal_orientation_tolerance(0.1) # エンドエフェクタ姿勢の許容誤差
    joint_constraint = sc.make_joint_constraint(move_group, 3.142)
    move_group.set_path_constraints(joint_constraint) # 関節角の制限
    rospy.loginfo("joint_constraints applied: %s", joint_constraint)
    move_group.set_planning_time(0.5) # タイムアウトの設定
    rospy.loginfo("plan start")
    flag, plan, time, error = move_group.plan() # 計画
    
    if flag:
        # 目標関節角を取得
        goal_joints = plan.joint_trajectory.points[-1].positions
        if goal_joints[0] > 3.142 or goal_joints[0] < -3.142:
            print("!-----out of range-----!")
            return None

        # ゴールの生成
        pub_msg = FollowJointTrajectoryGoal()
        pub_msg.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = goal_joints
        point.time_from_start = rospy.Duration(1)
        pub_msg.trajectory.points.append(point)
        rospy.loginfo(pub_msg.trajectory.points[0].positions)

        client.send_goal(pub_msg, done_cb=goal_callback)
        
    else:
        print("!-----planning failed-----!")
    # 現在位置の表示
    new_pose = move_group.get_current_pose().pose
    print(new_pose)    
    
if __name__ == "__main__":
    time = 10
    if len(sys.argv) == 2:
        time = int(sys.argv[1])
    rospy.sleep(time)
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("me6_arm")

    rospy.Subscriber("magician_move", Pose, callback)
    rospy.Subscriber("magician_move_jointangles", JointState, ma.move_by_jointangles)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
