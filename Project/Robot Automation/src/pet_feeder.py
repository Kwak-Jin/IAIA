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
from ur_python.msg import pet_info

DEG2RAD = pi/180
RAD2DEG = 180/pi


feed_info = {   "pt_A_top"   :   {  "tabby cat"         : np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD   ,
                                    "Golden Retriever"  : np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD   },
                "pt_A_bottom":   {  "tabby cat"         : np.array([30.13, -87.14, 129.82, -122.63, -89.81, -57.64])*DEG2RAD  ,
                                    "Golden Retriever"  : np.array([30.13, -87.14, 129.82, -122.63, -89.81, -57.64])*DEG2RAD  },
                "pt_B_top"   :   {  "tabby cat"         : np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD    ,
                                    "Golden Retriever"  : np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD    },
                "pt_B_bottom":   {  "tabby cat"         : np.array([112.26, -85.02, 112.84, -117.77, -89.80, 19.29])*DEG2RAD   ,
                                    "Golden Retriever"  : np.array([112.26, -85.02, 112.84, -117.77, -89.80, 19.29])*DEG2RAD   },
                
                "quantity"  :   {   "tabby cat"         : 2,
                                    "Golden Retriever"  : 4},

                "up_A"       :   {  "rel_xyz"           : [0.0, 0.0, 0.213]     ,
                                    "rel_rpy"           : [0.0, 0.0, 0.0]      },
                "down_A"     :   {  "rel_xyz"           : [0.0, 0.0, -0.213]    ,
                                    "rel_rpy"           : [0.0, 0.0, 0.0]      },
                "up_B"       :   {  "rel_xyz"           : [0.0, 0.0, 0.178]     ,
                                    "rel_rpy"           : [0.0, 0.0, 0.0]      },
                "down_B"     :   {  "rel_xyz"           : [0.0, 0.0, -0.178]    ,
                                    "rel_rpy"           : [0.0, 0.0, 0.0]       }
            }


class PetFeederNode():
    def __init__(self):
        # rospy.init_node('pet_feeder', anonymous=True) # 노드 초기화 및 이름 설정

        # Subscriber
        self.sub_pet_class = rospy.Subscriber("pet_classifier/pet_info", pet_info, self.feed)  # camera/image_raw 토픽에서 Image 메시지 수신

        # Robot initialization
        self.robot = MoveGroupPythonInterface(real="sim")
        self.robot.move_to_standby()
        # self.robot.grip_off()
    
    def feed(self, pet_info):
        
        for i in range(feed_info["quantity"][pet_info.name]):
            
            # point A
            # go_to_pose_abs(self, absolute_xyz, absolute_rpy)
            self.robot.go_to_joint_abs(feed_info["pt_A_top"][pet_info.name])
            self.robot.go_to_pose_rel(feed_info["down_A"]['rel_xyz'], feed_info["down_A"]['rel_rpy'])
            # self.robot.grip_on()
            self.robot.go_to_pose_rel(feed_info["up_A"]['rel_xyz'], feed_info["up_A"]['rel_rpy'])

            # point B
            self.robot.go_to_joint_abs(feed_info["pt_B_top"][pet_info.name])
            self.robot.go_to_pose_rel(feed_info["down_B"]['rel_xyz'], feed_info["down_B"]['rel_rpy'])
            # self.robot.grip_off()
            self.robot.go_to_pose_rel(feed_info["up_B"]['rel_xyz'], feed_info["up_B"]['rel_rpy'])

        self.robot.move_to_standby()
            
    def run(self):
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
    try:
        pet_feeder = PetFeederNode()
        pet_feeder.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
