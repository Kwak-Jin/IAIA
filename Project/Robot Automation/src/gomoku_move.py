#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy
# from tf.transformations import *
from math import tau, pi
import numpy as np
from std_msgs.msg import Int32MultiArray
from move_group_python_interface import MoveGroupPythonInterface
from ur_python.msg import capture_flag

import sys
import os   
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, '..', 'src')
sys.path.insert(0, src_dir)
#time variable used for count down
time = 5
target_pose_abs_xyz = [-0.190, 0.3025, -0.305]    # Stone Placement position
target_pose_abs_rpy = [0.00, 0.00, 0.00]         # Default rotation angle(0,0,0)
aaf_grip_pose_xyz = [-0.20, 0.00, 0.0]             # Relative Position after vacuum  (XYZ)
af_grip_pose_xyz = [0.00, 0.00, 0.233]             # Relative Position after vacuum  (XYZ)
grip_pose_xyz = [00.0, -0.00, -0.023]            # Relative Position vacuum        (XYZ)
grip_pose_rpy = [0.00, 0.00, 0.00]               # Default Roll pitch yaw (rad)
DEG2RAD = pi/180.0
grip_abs_point = [63.11*DEG2RAD, -80.75*DEG2RAD, 90.75*DEG2RAD, -101.50*DEG2RAD, -90.17*DEG2RAD, -39.7*DEG2RAD]

init_pose_joints = [pi*0.472, -pi*0.534, pi*0.064, -pi*0.127, -pi*0.493, -pi*0.021]  # Camera position
waypoint_init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]              # Original position

def count_time():
    # Time count using variable time
    print("\r")
    for k in range(time, 0, -1):
        print(f"{k}s left", end="\r")
        rospy.sleep(1)
    print("\r")
    print("Times out")
    print("\r")


class UR5e_Move_With_Camera():
    def __init__(self):
        self.ur5e = MoveGroupPythonInterface(real="real", gripper="Vacuum")
        init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi  원위치
        self.ur5e.go_to_joint_abs(init_pose_joints)
        
        self.msg_capture_flag = capture_flag()

        # Subscribe from Gomoku algorithm(omoks.cpp)
        self.sub_object_info = rospy.Subscriber("Ai_Stone_Coord", Int32MultiArray, self.detection_callback)
        # Publish
        self.pub_capture_flag = rospy.Publisher("capture_flag", capture_flag, queue_size=10)

        self.flag_recv_msg = False
        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_x_prev = 0
        self.cmd_y_prev = 0
        print("Initialization is completed!")

    def go_grip_stone(self,stone_pos1):
        self.ur5e.go_to_pose_rel(stone_pos1, grip_pose_rpy)
        rospy.sleep(0.5)
        self.ur5e.go_to_pose_rel(grip_pose_xyz, grip_pose_rpy)
        self.ur5e.grip_on()
        
    #Get x,y coordinates from Gomoku algorithm(c++)
    def detection_callback(self, data):
        print(f"message received: {data.data[0]}, {data.data[1]}")
        self.flag_recv_msg = True
        self.cmd_x = data.data[0]
        self.cmd_y = data.data[1]

    def go_cam_position(self):
        self.ur5e.go_to_pose_rel(af_grip_pose_xyz, grip_pose_rpy)
        #self.ur5e.go_to_pose_rel(aaf_grip_pose_xyz, grip_pose_rpy)
        #self.ur5e.go_to_joint_abs(waypoint_init_pose_joints)     # 원위치
        self.ur5e.go_to_joint_abs(init_pose_joints)      # Camera point

    def publish_capture_flag(self):
        self.msg_capture_flag = True
        self.pub_capture_flag.publish(self.msg_capture_flag)
        print(f"Capture Flag Published: {self.msg_capture_flag}")

    def run(self):
        x_jump = 0.039      # Stone tray x jump
        y_jump = 0.03945    # Stone tray y jump
        grid_jump = 0.0355  # Board Grid distance
        # Stone Tray Position indexx
        cnt_x = 0
        cnt_y = 0        
        while not rospy.is_shutdown():  
            self.cmd_x_prev = self.cmd_x
            self.cmd_y_prev = self.cmd_y      
            be_grip_pose_xyz = [0.103 + cnt_y*x_jump, -0.198 + cnt_x*y_jump, -0.2]               # Position before vacuum (XYZ)

            # Move to placed stone to grip
            self.ur5e.go_to_joint_abs(grip_abs_point)
            self.go_grip_stone(be_grip_pose_xyz)

            # Move to designated location to take a picture
            self.go_cam_position()
            
            if self.flag_recv_msg:
                
                if(self.cmd_x == - 100 and self.cmd_y == 100):
                    print("Finish Game!!\r\n")
                    break
            # else:
            #     print("Finish the game, You are a loser. Please think fastly")
            #     break
            
            self.flag_recv_msg = False

            count_time()
            self.publish_capture_flag()

            #Subscribed data from gomoku algorithm comes here
            while(self.cmd_x_prev==self.cmd_x and self.cmd_y_prev == self.cmd_y):
                continue
            if(self.cmd_x==-100 and self.cmd_y == -100):
                print("Human won!!!")
                break
            if(self.cmd_x==100 and self.cmd_y == 100):
                print("AI won!!!")
                break
            self.ur5e.go_to_joint_abs(waypoint_init_pose_joints)     # Waypoint to the stone placement
            target_pose_abs_xyz_go = [target_pose_abs_xyz[0]+self.cmd_x*grid_jump, target_pose_abs_xyz[1]-self.cmd_y*grid_jump,-0.304]  # 판에 두기
            rospy.sleep(0.5)
            self.ur5e.go_to_pose_rel(target_pose_abs_xyz_go, target_pose_abs_rpy)
            self.ur5e.grip_off()
            self.ur5e.go_to_joint_abs(waypoint_init_pose_joints)     # 원위치

            # Update stone tray position indices 
            cnt_x = cnt_x + 1
            if cnt_x == 10:
                cnt_x = 0
                cnt_y = cnt_y + 1
            if cnt_y == 3:
                # Reset tray position to the original position
                cnt_x = 0
                cnt_y = 0

def main():
    try:
        UR5e = UR5e_Move_With_Camera()
        UR5e.run()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()