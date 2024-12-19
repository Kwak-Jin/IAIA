#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ur_python.msg import object_info

class StoneMessageCheck:

    def __init__(self):
        rospy.init_node('Stone_Message_Check', anonymous=True)  # 노드 이름 "Stone Message Check"로 초기화
        self.bridge = CvBridge()  # cv_bridge 객체 생성

        # Subscribe to the "Stone Message Receiver" topic
        self.sub_capture_flag = rospy.Subscriber("Stone_Message_Receiver", object_info, self.found_new_stone)

        rospy.loginfo("Initialized Complete!")

    def found_new_stone(self, data):
        # Print the x and y values from the received new_stone message
        rospy.loginfo(f"Received new stone at coordinates: x = {data.x}, y = {data.y}")

    def run(self):
        rospy.loginfo("Stone Message Check Node is running.")
        rospy.spin()  # Keep the node running to listen for new messages

if __name__ == '__main__':
    try:
        # Create and run the StoneMessageCheck node
        stone_checker = StoneMessageCheck()  # StoneMessageCheck 객체 생성
        stone_checker.run()  # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
