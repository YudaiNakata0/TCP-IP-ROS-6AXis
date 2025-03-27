#!/usr/bin/env python3

import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint

def make_joint_constraint(group_name, limit_angle):
    constraint = Constraints()
    joint_names = group_name.get_active_joints()
    for joint_name in joint_names:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = limit_angle
        joint_constraint.tolerance_below = -limit_angle
        joint_constraint.weight = 1.0
        constraint.joint_constraints.append(joint_constraint)
    return constraint
