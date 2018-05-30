#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')

if __name__=='__main__':

    rospy.sleep(5.0)

    try:
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # 少し後ろに移動
        pos = omni_base.pose
        omni_base.go(pos[0] + 0.5, pos[1], pos[2], _MOVE_TIMEOUT)
    except:
        rospy.logerr('fail to move')
        sys.exit()

    try:
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
    except:
        rospy.logerr('fail to grasp')
        sys.exit()
