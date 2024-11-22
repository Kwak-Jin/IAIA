#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import numpy as np
from math import pi

from move_group_python_interface import MoveGroupPythonInterface

DEG2RAD = pi/180
RAD2DEG = 180/pi

def main():
    try:
        print("grip")
        ur5e = MoveGroupPythonInterface(real="sim")
        
        ur5e.move_to_standby()
        # ur5e.grip_off()
        
        # go to A top
        pos_A = np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD
        ur5e.go_to_joint_abs(pos_A)

        # go to A bottom
        rel_xyz = [0.0, 0.0, -0.213]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        # grip
        # ur5e.grip_on()

        # go to A top
        rel_xyz = [0.0, 0.0, 0.213]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)


        # go to B top
        pos_B = np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD 
        ur5e.go_to_joint_abs(pos_B)

        # go to B bottom
        rel_xyz = [0.0, 0.0, -0.178]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        # grip
        # ur5e.grip_off()

        # go to B top
        rel_xyz = [0.0, 0.0, 0.178]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        ur5e.move_to_standby()

        print("complete")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("Shut down by Key Interrupt")
        return


if __name__ == '__main__':
    main()  