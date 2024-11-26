#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy
import numpy as np
from move_group_python_interface import MoveGroupPythonInterface
from math import tau, pi

DEG2RAD = tau / 360.0

grid_jump = 0.0355


def main():
    try:
        
        ur5e = MoveGroupPythonInterface(real="real", gripper="Vacuum")
        init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        target_pose_abs_xyz = [-0.003, -0.05, -0.305]
        target_pose_abs_rpy = [0.00, 0.00, 0.00]
        print("============ Moving initial pose using a joint state goal ...")
        # ur5e.move_to_standby()
        ur5e.go_to_joint_abs(init_pose_joints)
        grip_pose_joints = [0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]
        input("============ Press `Enter` to execute a movement using a absolute pose 3...")
        cnt = 1
        while 1:            
            ur5e.go_to_joint_abs(grip_pose_joints)
            ur5e.grip_on()
            ur5e.go_to_joint_abs(init_pose_joints)
            print(f"Turn = {cnt} AI's turn")
            x,y = input("============ Press 2 Integer values for Gomoku...").split()

            if x == -1000 and y == -1000:
                break
            # if (x)>7 or x<-7 (y)>7 or y<-7:
            #     print("Wrong position")
            #     continue
            target_pose_abs_xyz_go = [target_pose_abs_xyz[0]+float(x)*grid_jump, target_pose_abs_xyz[1]+float(y)*grid_jump,-0.305]
            ur5e.go_to_pose_rel(target_pose_abs_xyz_go, target_pose_abs_rpy)
            ur5e.grip_off()
            cnt = cnt + 1


        print("============ Moving initial pose using a joint state goal ...")
        # ur5e.move_to_standby()
        init_pose_joints = [tau/8, -tau/8, tau/8, -tau/8, -tau/8, 0.0]
        init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        ur5e.go_to_joint_abs(init_pose_joints)

        # input("============ Press `Enter` to execute a movement using a absolute pose 3...")
        # target_pose_abs_xyz = [-0.0, -0.2, -0.0]
        # target_pose_abs_rpy = [0.00, 0.00, 0.00]
        # ur5e.go_to_pose_rel(target_pose_abs_xyz, target_pose_abs_rpy)

        # input("============ Press `Enter` to execute a movement using a absolute pose 3...")
        # ur5e.grip_off()


        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 1...")
        # joint_rel = [0, 1/11 * tau, 0, - 1/11 * tau, 0, 0]          # tau = 2 * pi
        # ur5e.go_to_joint_rel(joint_rel)
        
        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 1...")
        # joint_rel = [33.45*tau/360, -81.3*tau/360, 110*tau/360,-120*tau/360, -90*tau/360, -20*tau/360]          # tau = 2 * pi
        # ur5e.go_to_joint_abs(joint_rel)
        
        # input("============ Press `Enter` to use gripper...")
        # ur5e.grip_on()

        # input("============ Press `Enter` to Initial Position...")
        # init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        # ur5e.go_to_joint_abs(init_pose_joints)
        
        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 1...")
        # joint_rel = [79.42*tau/360, -97.50*tau/360, 149.00*tau/360,-142*tau/360, -94.15*tau/360, -18.33*tau/360]          # tau = 2 * pi
        # ur5e.go_to_joint_abs(joint_rel)
        
        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 2_1...")
        # ur5e.grip_off()
        # print("End!!")

        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 2_1...")
        # ur5e.grip_off()



        
        print("============ complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("Shut down by Key Interrupt")
        return

if __name__ == "__main__":
    main()
