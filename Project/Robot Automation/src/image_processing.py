#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리
import numpy as np

from ur_python.msg import object_info, robot_state

class ImageProcessingNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("camera/image_raw",Image,self.callback)  # camera/image_raw 토픽에서 Image 메시지 수신
        self.flag_find = False
        self.flag_send_msg = False
        self.pub_object_info = rospy.Publisher("object_info", object_info, queue_size=10)
        self.msg_object_info = object_info()
        self.msg_robot_state = robot_state()
        rospy.Subscriber("robot_state", robot_state, self.recv_robot_state)

    def callback(self,data):
        try:
            # 수신된 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  
            height = cv_image.shape[0]
            width  = cv_image.shape[1]
            
            image = np.asanyarray(cv_image[int(height/2 - 240) : int(height/2 + 240), int(width/2 - 240): int(width/2 + 240)])     # ROI
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_bound = np.array([0, 150, 100])
            upper_bound = np.array([10, 255, 255])

            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            result = cv2.bitwise_and(image, image, mask=mask)

            # if not self.flag_find:
            # mask에서 Contour를 특정
            c, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in c:
                
                if cv2.contourArea(contour) > 10000:
                    (x, y, w, h) = cv2.boundingRect(contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    self.flag_find = True
                else:
                    self.flag_find = False
            
            if self.flag_find:
                if not self.flag_send_msg:
                    self.convert_img2real(x, y, w, h)
                    self.pub_object_info.publish(self.msg_object_info)
                    self.flag_send_msg = True
                    print(f"message sended: {self.msg_object_info.x:.2f}, {self.msg_object_info.y:.2f}")

            # cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
            cv2.imshow("image", image)
            # cv2.imshow("mask", mask)
            # cv2.imshow("result", result)
            cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

        except CvBridgeError as e:
            print(e)

    def convert_img2real(self, x, y, w, h):
        cen_x = x + w/2
        cen_y = y + h/2
        real_x = cen_x / 1000 + 0.2
        real_y = cen_y / 1000 + 0.1
        self.msg_object_info.x = real_x
        self.msg_object_info.y = real_y

    def recv_robot_state(self, data):
        self.msg_robot_state.move = data.move
        if self.msg_robot_state.move == 0:
            self.flag_send_msg = False
        
            
    def run(self):
        rospy.init_node('Image_Processing', anonymous=True) # 노드 초기화 및 이름 설정
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
    try:
        image_processing = ImageProcessingNode()    # ImageProcessingNode 클래스의 인스턴스 생성
        image_processing.run()                      # 노드 실행
    except rospy.ROSInterruptException:
        pass
