#!/usr/bin/env python3

import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint

def make_joint_constraint(group_name, tolerance):
    constraint = Constraints()
    joint_names = group_name.get_active_joints()
    joint_angles = group_name.get_current_joint_values()
    i = 0
    for joint_name in joint_names:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = joint_angles[i]
        joint_constraint.tolerance_above = tolerance
        joint_constraint.tolerance_below = tolerance
        joint_constraint.weight = 1.0
        constraint.joint_constraints.append(joint_constraint)
        i += 1
    return constraint
