#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from ur_python.msg import object_info

class ObjectInformationPublisher():
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('object_information_publisher', anonymous=True)
        
        # Publisher 생성: 'object_info' 토픽으로 object_info 메시지를 퍼블리시
        self.pub_object_info = rospy.Publisher('object_info', object_info, queue_size=10)
        
        # object_info 메시지 초기화
        self.msg_object_info = object_info()
        
        print("Object Information Publisher Initialized.")
        print("Press '1' to send object information.")

    def publish_object_info(self):
        # object_info 메시지의 데이터 설정
        self.msg_object_info.x = 0.5  # 예시 값
        self.msg_object_info.y = 0.5  # 예시 값
        # self.msg_object_info.z = 0.2  # 예시 값
        
        # 퍼블리시
        self.pub_object_info.publish(self.msg_object_info)
        print(f"Object Info Published: {self.msg_object_info}")

    def run(self):
        while not rospy.is_shutdown():
            user_input = input("Enter '1' to publish object info: ")

            if user_input == '1':
                # object_info 메시지 퍼블리시
                self.publish_object_info()

            else:
                print("Invalid input. Please enter '1' to send object info.")

if __name__ == "__main__":
    try:
        # ObjectInformationPublisher 객체 생성 후 실행
        object_info_publisher = ObjectInformationPublisher()
        object_info_publisher.run()

    except rospy.ROSInterruptException:
        pass
