#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리
import numpy as np

import torch
from torchvision import models
from torchvision.models import AlexNet_Weights
from PIL import Image as PILImage
from torchvision import transforms
import torch.nn.functional as F

import json
import urllib

from ur_python.msg import robot_state, pet_info

transforms = transforms.Compose([
                transforms.Resize(256),
                transforms.CenterCrop(224),
                transforms.ToTensor(),           
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
            )])



# ImageNet 클래스 레이블을 로드하는 함수
def load_imagenet_labels():
    url = "https://raw.githubusercontent.com/anishathalye/imagenet-simple-labels/master/imagenet-simple-labels.json"
    filename = "imagenet_classes.json"
    urllib.request.urlretrieve(url, filename)
    
    with open(filename, 'r') as f:
        return json.load(f)

class PetClassifierNode():
    def __init__(self, labels):
        rospy.init_node('pet_classifier', anonymous=True) # 노드 초기화 및 이름 설정
        self.bridge = CvBridge()

        
        self.bAction = False
        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action)  # camera/image_raw 토픽에서 Image 메시지 수신
        self.sub_robot = rospy.Subscriber("robot_state", robot_state, self.chk_robot)  # camera/image_raw 토픽에서 Image 메시지 수신
        
        # Publisher
        self.pub_pet_info = rospy.Publisher('pet_classifier/pet_info', pet_info, queue_size=10)

        # Model init
        self.model = models.alexnet(weights=AlexNet_Weights.DEFAULT)
        self.model.eval()
        self.labels = labels

        # Message Control Variables
        self.bSend = False
        self.bAction = False
        self.pet_info = pet_info()
        self.thresh_cnt = 30
        self.chk_cnt = 0    
        self.pet_id_prev = 0
        self.pet_id_curr = 0
        

    def chk_robot(self,robot_state):
        if robot_state.move == 1:
            self.bAction = False
        else:
            self.bAction = True

    def action(self,data):
        if self.bAction:
            try:
                # 수신된 Image 메시지를 OpenCV 이미지로 변환
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 
                
                # OpenCV 이미지를 PIL 이미지로 변환
                pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
                
                # 이미지 전처리
                input_tensor = transforms(pil_image)
                input_batch = input_tensor.unsqueeze(0)  # 배치 차원 추가

                # model 예측
                with torch.no_grad():
                    output = self.model(input_batch)
                    percentages = F.softmax(output, dim=1)[0] * 100
                    _, indices = torch.sort(output, descending=True)
                    
                    label = self.labels[indices[0][0]]
                    

                    if label == "tabby cat":
                        self.pet_id_curr = 1
                    elif label == "Golden Retriever":
                        self.pet_id_curr = 2
                    else:
                        self.pet_id_curr = 0

                    if (self.pet_id_curr == self.pet_id_prev) and self.pet_id_curr != 0:
                        self.chk_cnt += 1
                    else:
                        self.chk_cnt = 0

                    if self.chk_cnt >= self.thresh_cnt:
                        self.chk_cnt = 0
                        self.bSend = True

                    if self.bSend:
                        self.pet_info.name = label
                        self.pub_pet_info.publish(self.pet_info)
                        self.bSend = False

                    self.pet_id_prev = self.pet_id_curr

                    rospy.loginfo("%s, %d, %d",label, self.pet_id_curr, self.chk_cnt)


                # cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
                # cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

            except Exception as e:
                print(e)

    def run(self):
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행


if __name__ == '__main__':
    try:
        # ImageNet 레이블 로드
        imagenet_labels = load_imagenet_labels()
        pet_classifier = PetClassifierNode(imagenet_labels)       # CameraNode 객체 생성
        pet_classifier.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass