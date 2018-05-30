#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
from hsrb_interface import geometry
import rospy
_MOVE_TIMEOUT=60.0
_GRASP_FORCE=0.01
_GRIPPER_OPEN=0.12
_RED_BLOCK_TF='red_object'
_MAP_TF='map'
_HAND_OFFSET=-0.055
_MOVE_POS=(10, -1.0, 1.57)
_POST_POSE=(0.6, 0.45, 0.4, -1.57, -1.57, 0)

if __name__=='__main__':
    robot = hsrb_interface.Robot()
    omni_base = robot.get('omni_base')
    whole_body = robot.get('whole_body')
    gripper = robot.get('gripper')

    whole_body.move_to_neutral()
    omni_base.go_abs(*_MOVE_POS)

    while not rospy.is_shutdown():
        gripper.set_distance(_GRIPPER_OPEN)
        whole_body.move_end_effector_pose(
            pose=geometry.pose(z=_HAND_OFFSET),
            ref_frame_id=_RED_BLOCK_TF)
        gripper.apply_force(_GRASP_FORCE)
        rospy.sleep(1.0)
        whole_body.move_to_neutral()
        whole_body.move_end_effector_pose(
            pose=geometry.pose(
                x=_POST_POSE[0],
                y=_POST_POSE[1],
                z=_POST_POSE[2],
                ei=_POST_POSE[3],
                ej=_POST_POSE[4],
                ek=_POST_POSE[5]),
            ref_frame_id=_MAP_TF)
        gripper.set_distance(_GRIPPER_OPEN)
        whole_body.move_to_neutral()
        omni_base.go_abs(*_MOVE_POS)
