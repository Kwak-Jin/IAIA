#! /usr/bin/env python3
#-*- coding:utf-8 -*- 

from move_group_python_interface import MoveGroupPythonInterface
from math import tau
import rospy

def main():
    try:

        ur5e = MoveGroupPythonInterface()

        input("============ Press `Enter` to execute a movement using a joint state goal ...")
        target_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        ur5e.go_to_joint_abs(target_joints)

        input("============ Press `Enter` to grip on ...")
        ur5e.grip_on()

        input("============ Press `Enter` to grip off ...")
        ur5e.grip_off()

        print("============ Python tutorial demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("Shut down by Key Interrupt")
        return

if __name__ == '__main__':
    main()