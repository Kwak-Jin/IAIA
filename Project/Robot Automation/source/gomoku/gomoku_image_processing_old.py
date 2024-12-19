#!/usr/bin/env python3
#-*- coding:utf-8 -*- 


import rospy
from sensor_msgs.msg import Image   
from cv_bridge import CvBridge, CvBridgeError      
import cv2                          
import numpy as np

from ur_python.msg import new_stone, capture_finished_flag


class StoneDetectionNode():
    def __init__(self):
        rospy.init_node('gomoku_image_processing', anonymous=True)

        self.bridge = CvBridge()
        self.detect_stone = False  # 이미지 처리 여부를 나타내는 변수

        # Subscriber: capture_finished_flag를 수신
        self.sub_captured_flag = rospy.Subscriber("capture_completed", capture_finished_flag, self.find_new_stone)
    
        # Publisher: new_stone을 퍼블리시
        self.pub_new_stone = rospy.Publisher('gomoku_image_processing/new_stone', new_stone, queue_size=10)

        # 카메라 내부 파라미터
        fx = 1356.566963  # 초점 거리 x
        fy = 1356.566963  # 초점 거리 y
        cx = 640.000000   # 중심점 x
        cy = 360.000000   # 중심점 y

        # 왜곡 계수
        k1 = 0.143895
        k2 = -0.539060
        p1 = -0.002309
        p2 = 0.005301

        # 카메라 행렬
        self.K = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0, 0, 1]])

        self.dist_coeffs = np.array([k1, k2, p1, p2])

        # stone = 1: white, stone = 0: black
        self.stone = 0

        print("Initialized Complete!")
    

    def find_new_stone(self, capture_finished_flag):
        if capture_finished_flag.capture_finished == False:
            self.detect_stone = False
        else:
            self.detect_stone = True
            rospy.loginfo("Detect the Stone")
            self.processing()  # 이미지 캡처 함수 호출


    def processing(self):
        if self.detect_stone == True:

            img_dir = '/home/gyeonheal/catkin_ws/src/ur_python/src/captured_images/5.jpg'
            image = cv2.imread(img_dir)
            
            # Camera Calibration
            image = cv2.undistort(image, self.K, self.dist_coeffs)

            # ------------------------------------------------------- Board Detecting & ROI --------------------------------------------------------

            src_points = np.array([
                [361, 21],  [1022, 15], [1108, 702], [315, 716]   
            ], dtype='float32')

            width, height = 400, 400  
            dst_points = np.array([
                [0, 0],              
                [width - 1, 0],
                [width - 1, height - 1],  
                [0, height - 1]     
            ], dtype='float32')

            M = cv2.getPerspectiveTransform(src_points, dst_points)

            warped_img = cv2.warpPerspective(image, M, (width, height))

            # 테두리 생성
            border_thickness = 20

            new_height = height + 2 * border_thickness
            new_width = width + 2 * border_thickness
            # stone = 1: white, stone = 0: black
            stone = 0
            green_color = (0, 255, 0)
            bordered_img = np.zeros((new_height, new_width, 3), dtype=np.uint8)
            bordered_img[:] = green_color  
            bordered_img[border_thickness:border_thickness + height, border_thickness:border_thickness + width] = warped_img

            # Image Copying            
            board_img = np.zeros_like(bordered_img)
            mask = np.ones(bordered_img.shape, dtype=np.uint8)
            cv2.copyTo(bordered_img, mask, board_img)

            # ------------------------------------------------------- Coordinate Calculating --------------------------------------------------------

            # 포인트 생성 영역 (테두리 제외)
            h, w, c = board_img.shape

            # 테두리를 제외한 영역에서 포인트 생성
            cross_x = []
            cross_y = []

            gap = 47

            for i in range(0, 15):
                cross_x.append(gap + (w - 2 * gap) / 14 * i)
                cross_y.append(gap + (h - 2 * gap) / 14 * i)

            coordinates = []

            for i in cross_x:
                for j in cross_y:
                    coordinates.append((i, j))

            # 포인트 그리기
            for coord in coordinates:
                x, y = coord
                cv2.circle(board_img, (int(x), int(y)), 2, (0, 0, 255), -1)

            # ------------------------------------------------------- Stone Detecting --------------------------------------------------------

            low_w = np.array([175, 150, 150])
            high_w = np.array([255, 255, 255])

            low_b = np.array([0, 0, 0])
            high_b = np.array([75, 75, 75])

            radius = 5

            white_idx = []
            black_idx = []

            for i in range(0,15):
                for j in range(0,15):
                    x = cross_x[i]
                    y = cross_y[j]

                    x_min = max(int(x - radius), 0)
                    x_max = min(int(x + radius), bordered_img.shape[1])
                    y_min = max(int(y - radius), 0)
                    y_max = min(int(y + radius), bordered_img.shape[0])

                    region = bordered_img[y_min:y_max, x_min:x_max]
                    avg_bgr = np.mean(region, axis=(0, 1)) 
                    
                    if np.all(avg_bgr >= low_w) and np.all(avg_bgr <= high_w):
                        cv2.rectangle(board_img, (x_min, y_min), (x_max, y_max), (255, 0, 255), 2)
                        white_idx.append((i, j))

                    elif np.all(avg_bgr >= low_b) and np.all(avg_bgr <= high_b):
                        cv2.rectangle(board_img, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                        black_idx.append((i, j))

            cv2.imshow("Board Image", board_img)

            with open("/home/gyeonheal/catkin_ws/src/ur_python/src/user_idx.txt", "r") as file:
                user_idx = eval(file.read())

            # 사람이 놓은 돌 좌표
            new_stone_coord = [item for item in black_idx if item not in user_idx]
            print(f"new stone coord is : ", new_stone_coord)
            for coord in new_stone_coord:
                stone_msg = new_stone()
                stone_msg.x = coord[0]
                stone_msg.y = coord[1]

                self.pub_new_stone.publish(stone_msg)
                print(stone_msg)

            with open("/home/gyeonheal/catkin_ws/src/ur_python/src/user_idx.txt", "w") as file:
                if self.stone == 0:
                    file.write(f"{black_idx}")
                elif self.stone == 1:
                    file.write(f"{white_idx}")

    def run(self):
        rospy.loginfo("Image Processing Node is running")
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행
        
def main():
    try:
        image_processing = StoneDetectionNode()    # ImageProcessingNode 클래스의 인스턴스 생성
        image_processing.run()                      # 노드 실행
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()