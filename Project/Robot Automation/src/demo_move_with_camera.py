#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy

from tf.transformations import *
from math import tau

from move_group_python_interface import MoveGroupPythonInterface
from ur_python.msg import object_info, robot_state

class UR5e_Move_With_Camera():
    def __init__(self):
        self.ur5e = MoveGroupPythonInterface(real="sim")

        init_pose_joints = [-tau/4, -tau/4, -tau/4, -tau/4, tau/4, 0.0]          # tau = 2 * pi
        self.ur5e.go_to_joint_state(init_pose_joints)
        self.msg_object_info = object_info()
        # self.msg_robot_state = robot_state()
        self.sub_object_info = rospy.Subscriber("object_info", object_info, self.detection_callback)
        # self.pub_robot_state = rospy.Publisher("robot_state", robot_state, queue_size=10)
        
        self.flag_recv_msg = False
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        
        print("Initialization is completed!")

    def detection_callback(self, data):
        # print(f"message received: {data.x}, {data.y}")
        self.flag_recv_msg = True
        self.cmd_x = data.x
        self.cmd_y = data.y

    def run(self):
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            if self.flag_recv_msg:
                print(f"message received: {self.cmd_x:.2f}, {self.cmd_y:.2f}")
                target_pose_abs_xyz = [self.cmd_x, self.cmd_y, 0.28]

                current_pose = self.ur5e.manipulator.get_current_pose().pose
                current_quat = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
                current_rpy = euler_from_quaternion(current_quat)
                target_pose_abs_rpy = current_rpy
                
                # self.msg_robot_state.move = 1
                # self.pub_robot_state.publish(self.msg_robot_state)
                self.ur5e.go_to_pose_abs(target_pose_abs_xyz, target_pose_abs_rpy)
                self.flag_recv_msg = False
                
                # self.msg_robot_state.move = 0
                # self.pub_robot_state.publish(self.msg_robot_state)

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