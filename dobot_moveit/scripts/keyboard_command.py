#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3, Pose
import rosgraph
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from mymodule import call_service


msg = """
Instruction:

---------------------------

 w     e     r     u       i        o
(+x)  (+y)  (+z)  (+roll) (+pitch) (+yaw)

 s     d     f     j       k        l
(-x)  (-y)  (-z)  (-roll) (-pitch) (-yaw)

b: clear error
n: enable robot
m: disable robot
v: stop

x, y, z: 0.02 [m]
roll, pitch, yaw: 10 [degrees]

Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

def eulertoquat(euler_angles):
        rot = Rot.from_euler("xyz", euler_angles, degrees=True)
        qua = rot.as_quat()
        msg = Pose()
        msg.position.x = msg.position.y = msg.position.z = 0.0
        msg.orientation.x = qua[0]
        msg.orientation.y = qua[1]
        msg.orientation.z = qua[2]
        msg.orientation.w = qua[3]
        return msg.orientation
        
if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "")
        print(msg)

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        # land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        # halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        # start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        # takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        # force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/magician_move', Pose, queue_size=1)

        x_move = 0.02 # [m]
        y_move = 0.02
        z_move = 0.02
        roll_rot = 10 # [degrees]
        pitch_rot = 10
        yaw_rot = 10

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        try:
                while(True):
                        nav_msg = Pose()
                        nav_msg.position.x = 0.0
                        nav_msg.position.y = 0.0
                        nav_msg.position.z = 0.0
                        nav_msg.orientation.x = 0.0
                        nav_msg.orientation.y = 0.0
                        nav_msg.orientation.z = 0.0
                        nav_msg.orientation.w = 1.0
                        euler_angles = [0.0, 0.0, 0.0]
                        # nav_msg.control_frame = FlightNav.WORLD_FRAME
                        # nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        # if key == 'l':
                        #         land_pub.publish(Empty())
                        #         msg = "send land command"
                        # if key == 'r':
                        #         start_pub.publish(Empty())
                        #         msg = "send motor-arming command"
                        # if key == 'h':
                        #         halt_pub.publish(Empty())
                        #         msg = "send motor-disarming (halt) command"
                        # if key == 'f':
                        #         force_landing_pub.publish(Empty())
                        #         msg = "send force landing command"
                        # if key == 't':
                        #         takeoff_pub.publish(Empty())
                        #         msg = "send takeoff command"
                        # if key == 'x':
                        #         motion_start_pub.publish()
                        #         msg = "send task-start command"
                        if key == 'w':
                                nav_msg.position.x = x_move
                                nav_pub.publish(nav_msg)
                                msg = "send +x command"
                        if key == 's':
                                nav_msg.position.x = -x_move
                                nav_pub.publish(nav_msg)
                                msg = "send -x command"
                        if key == 'e':
                                nav_msg.position.y = y_move
                                nav_pub.publish(nav_msg)
                                msg = "send +y command"
                        if key == 'd':
                                nav_msg.position.y = -y_move
                                nav_pub.publish(nav_msg)
                                msg = "send -y command"
                        if key == 'r':
                                nav_msg.position.z = z_move
                                nav_pub.publish(nav_msg)
                                msg = "send +z command"
                        if key == 'f':
                                nav_msg.position.z = -z_move
                                nav_pub.publish(nav_msg)
                                msg = "send -z command"
                        if key == 'u':
                                euler_angles[0] = roll_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send +roll command" 
                        if key == 'j':
                                euler_angles[0] = -roll_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send -roll command"
                        if key == 'i':
                                euler_angles[1] = pitch_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send +pitch command"
                        if key == 'k':
                                euler_angles[1] = -pitch_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send -pitch command"
                        if key == 'o':
                                euler_angles[2] = yaw_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw command"
                        if key == 'l':
                                euler_angles[2] = -yaw_rot
                                nav_msg.orientation = eulertoquat(euler_angles)
                                nav_pub.publish(nav_msg)
                                msg = "send -yaw command"
                        if key == 'b':
                                call_service.call_ClearError()
                                msg = "send clearerror command"
                        if key == 'n':
                                call_service.call_EnableRobot()
                                msg = "send enable command"
                        if key == 'm':
                                call_service.call_DisableRobot()
                                msg = "send disable command"
                        if key == 'v':
                                stop_pub = rospy.Publisher("/magician_move_stop", Empty, queue_size=10)
                                stop_msg = Empty()
                                stop_pub.publish(stop_msg)
                                msg = "send stop command"

                        # if key == '[':
                        #         nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                        #         nav_msg.target_vel_z = z_vel
                        #         nav_pub.publish(nav_msg)
                        #         msg = "send +z vel command"
                        # if key == ']':
                        #         nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                        #         nav_msg.target_vel_z = -z_vel
                        #         nav_pub.publish(nav_msg)
                        #         msg = "send -z vel command"
                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


