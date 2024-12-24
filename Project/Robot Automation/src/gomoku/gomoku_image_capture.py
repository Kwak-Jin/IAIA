#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ur_python.msg import capture_flag, capture_finished_flag

class CameraNode:

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        self.bridge = CvBridge()  # cv_bridge 객체 생성
        with open("/home/gyeonheal/catkin_ws/src/ur_python/src/user_idx.txt", "w") as file:
            file.write("")
        # 카메라 연결 (카메라 번호는 파라미터에서 받음, 기본값 0)
        camera_number = rospy.get_param('~camera_number', 0)
        self.cap = cv2.VideoCapture(camera_number)  # 카메라 열기
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera")
        rospy.loginfo(f"Camera number received: {camera_number}")

        self.msg_capture_finished_flag = capture_finished_flag()

        # 저장할 이미지 경로 설정
        self.save_path = "/home/gyeonheal/catkin_ws/src/ur_python/src/captured_images/"
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)  # 경로가 없다면 생성

        # Subscribe
        self.sub_capture_flag = rospy.Subscriber("capture_flag", capture_flag, self.chk_robot)

        # Publish
        self.pub_capture_finished_flag = rospy.Publisher("capture_completed", capture_finished_flag, queue_size=10)

        # flag_initialize
        self.capture_ready = False  # 기본적으로 카메라는 실행되지 않음

        print("Initialized Complete!")

    def chk_robot(self, capture_flag):
        # capture_flag가 False이면 카메라를 멈추고, True이면 카메라 실행하도록 설정
        if capture_flag.capture == False:
            self.capture_ready = False
            rospy.loginfo("Stopping camera.")
        else:
            self.capture_ready = True
            rospy.loginfo("Capture ready, capturing one frame...")

            # 카메라에서 한 프레임을 읽어서 저장
            if self.capture_ready:
                self.capture_image()

    def capture_image(self):
        # 카메라에서 한 프레임을 읽어오기
        ret, frame = self.cap.read()
        if ret:  # 이미지가 정상적으로 읽혀진 경우
            # 이미지 파일명 설정 (예: image_001.jpg)
            image_filename = os.path.join(self.save_path, "captured_image.jpg")
            
            # 이미지 저장
            cv2.imwrite(image_filename, frame)
            rospy.loginfo(f"Image saved: {image_filename}")

            # 메시지 퍼블리시
            self.msg_capture_finished_flag = True
            self.pub_capture_finished_flag.publish(self.msg_capture_finished_flag)

            rospy.loginfo("Capture finished and message published.")
        else:
            rospy.logwarn("Failed to capture image from the camera.")

    def run(self):
        rospy.loginfo("Camera Node is running.")
        rospy.spin()  # ROS 노드를 계속 실행하면서 콜백 대기

if __name__ == '__main__':
    try:
        camera = CameraNode()  # CameraNode 객체 생성
        camera.run()  # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
