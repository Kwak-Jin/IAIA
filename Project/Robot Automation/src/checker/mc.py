#!/usr/bin/env python3
#-*- coding:utf-8 -*-


import rospy
from std_msgs.msg import String
from move_group_python_interface import MoveGroupPythonInterface
from math import tau, pi
import rospy
import time

af_grip_pose_xyz = [0.00, 0.00, 0.2]             # Relative Position after vacuum  (XYZ)
grip_pose_xyz = [00.0, -0.00, -0.092]            # Relative Position vacuum        (XYZ)
grip_pose_rpy = [0.00, 0.00, 0.00]               # Default Roll pitch yaw (rad)

camera_joints = [pi*0.472, -pi*0.534, pi*0.064, -pi*0.127, -pi*0.493, -pi*0.021]  # Camera position
init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]              # Original position



def count_time(time):
        # Time count using variable time
        print("\r")
        for k in range(time, 0, -1):
            print(f"{k}s left", end="\r")
            rospy.sleep(1)
        print("\r")
        print("Times out")
        print("\r")


class UR5e_Checker_Move():
    def __init__(self):

        self.ur5e = MoveGroupPythonInterface(real="real", gripper="Vacuum")
        init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi  원위치
        self.ur5e.go_to_joint_abs(init_pose_joints)

        self.msg_read = rospy.Subscriber('/checkers/move', String, self.move_callback)
        self.pub = rospy.Publisher('/task_status', String, queue_size=10)

        self.flag_recv_msg = False
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0

        print("Initialization is completed!")


    def move_callback(self, msg):
        self.flag_recv_msg = True
        # 메시지 처리
        move_str = msg.data
        start, end = move_str.split('->')
        # 각 좌표를 ',' 기준으로 나누어 y, x 값을 추출
        self.y1, self.x1 = map(int, start.split(','))
        self.y2, self.x2 = map(int, end.split(','))
        # 출력
        print(f"y1 = {self.y1}, x1 = {self.x1}")
        print(f"y2 = {self.y2}, x2 = {self.x2}")

        time.sleep(4)
        self.flag_recv_msg = False


    def run(self):
        #self.ur5e.go_to_joint_abs(init_pose_joints)             # Original position
        self.ur5e.go_to_joint_abs(camera_joints)                # Camera point

        while not rospy.is_shutdown():

            jump = 0.0625
            init_point_x = 0.313-jump/2
            init_point_y = -0.192+jump/2


            target_pose_abs_xyz = [init_point_x, init_point_y]
            target_pose_abs_rpy = [0.00, 0.00, 0.00]

            # input("============ Press `Enter` to grip off on the designated coorination ...")


            if self.flag_recv_msg:   

                print("message recieved")
                time.sleep(3)
                self.ur5e.go_to_joint_abs(init_pose_joints)
                time.sleep(1)
                bf_target_pose_abs_xyz_go = [target_pose_abs_xyz[0]-self.x1*jump-self.y1*0.001, target_pose_abs_xyz[1]+self.y1*jump-self.x1*0.0012,-0.27]
                time.sleep(1)
                target_pose_abs_xyz_go = [0, 0, -0.043]
                time.sleep(1)
                self.ur5e.go_to_pose_rel(bf_target_pose_abs_xyz_go, target_pose_abs_rpy)        # Before Grip on position

                # input("============ Press `Enter` to grip off on the designated coordination ...")
                print("next step")
                time.sleep(3)
                self.ur5e.go_to_pose_rel(target_pose_abs_xyz_go, target_pose_abs_rpy)        # Grip on position
                time.sleep(1)
                self.ur5e.grip_on()
                time.sleep(1)
                self.ur5e.go_to_joint_abs(init_pose_joints)                                  # Original position

                # input("============ Press `Enter` to grip off on the designated coordination ...")
                print("next step")
                time.sleep(3)
                target_pose_abs_xyz_go = [target_pose_abs_xyz[0]-(self.x2)*jump-(self.y2)*0.001, target_pose_abs_xyz[1]+(self.y2)*jump-(self.x2)*0.0012,-0.313]
                time.sleep(1)   
                self.ur5e.go_to_pose_rel(target_pose_abs_xyz_go, target_pose_abs_rpy)        # Grip off position
                time.sleep(1)
                self.ur5e.grip_off()

                # input("============ LASTTTTTTT ============================================= ...")
                print("last step")
                time.sleep(3)
                self.ur5e.go_to_joint_abs(init_pose_joints)                                  # Original position
                time.sleep(3)
                self.ur5e.go_to_joint_abs(camera_joints)                                     # Camera point

                time.sleep(3)
                self.pub.publish("completed")
                print("messeage sent")

def main():
    try:
        UR5e = UR5e_Checker_Move()
        UR5e.run()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
