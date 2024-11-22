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

        # print("============ Moving initial pose using a joint state goal ...")
        # # ur5e.move_to_standby()
        # init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        # ur5e.go_to_joint_abs(init_pose_joints)

        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 1...")
        # joint_rel = [1/8 * tau, 0, 0, 0, 0, 0]          # tau = 2 * pi
        # ur5e.go_to_joint_rel(joint_rel)
        
        # input("============ Press `Enter` to execute a movement using a joint state goal(relative) 2...")
        # joint_rel = [-1/8 * tau, 0, 0, 0, 0, 0]          # tau = 2 * pi
        # ur5e.go_to_joint_rel(joint_rel)

        # input("============ Press `Enter` to execute a movement using a relative pose 1...")
        # target_pose_rel_xyz = [0.0, 0.0, 0.0]
        # target_pose_rel_rpy = [tau/16, 0, 0]
        # ur5e.go_to_pose_rel(target_pose_rel_xyz, target_pose_rel_rpy)
                
        # input("============ Press `Enter` to execute a movement using a relative pose 2...")
        # target_pose_rel_xyz = [0.0, 0.0, 0.0]
        # target_pose_rel_rpy = [-tau/16, 0, 0]
        # ur5e.go_to_pose_rel(target_pose_rel_xyz, target_pose_rel_rpy)
        
        # input("============ Press `Enter` to execute a movement using a absolute pose 3...")
        # target_pose_abs_xyz = [-0.13, 0.49, 0.47]
        # target_pose_abs_rpy = [-2.3939, -0.00, -0.00]
        # ur5e.go_to_pose_abs(target_pose_abs_xyz, target_pose_abs_rpy)
        # print("End!!")

        print("============ Moving initial pose using a joint state goal ...")
        # ur5e.move_to_standby()
        init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        ur5e.go_to_joint_abs(init_pose_joints)

        input("============ Press `Enter` to execute a movement using a absolute pose 3...")
        target_pose_abs_xyz = [-0.003, -0.05, -0.305]
        target_pose_abs_rpy = [0.00, 0.00, 0.00]
        ur5e.go_to_pose_rel(target_pose_abs_xyz, target_pose_abs_rpy)
        cnt = 0
        target_pose_abs_rpy = [0.00, 0.00, 0.00]
        while 1:            
            input("============ Press `Enter` to execute a movement `using a absolute pose 3...")
            if cnt<7:
                target_pose_abs_xyz = [0.0, grid_jump, 0.0]
            else:
                target_pose_abs_xyz = [grid_jump, 0.0008, 0.0]
            ur5e.go_to_pose_rel(target_pose_abs_xyz, target_pose_abs_rpy)
            cnt = cnt+1
            if cnt>15:
                cnt = 0
                break

        while 1:            
            input("============ Press `Enter` to execute a movement `using a absolute pose 4...")
            if cnt<7:
                target_pose_abs_xyz = [0.0, -grid_jump, 0.0]
            else:
                target_pose_abs_xyz = [-grid_jump, -0.0008, 0.0]
            ur5e.go_to_pose_rel(target_pose_abs_xyz, target_pose_abs_rpy)
            cnt = cnt+1
            if cnt>15:
                break

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
