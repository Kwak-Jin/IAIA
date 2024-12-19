#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

# 패키지 임포트
import rospy
from sensor_msgs.msg import Image   
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from omok import State
from pv_mcts import pv_mcts_action
from tensorflow.keras.models import load_model
import tensorflow as tf
from pathlib import Path
from threading import Thread

# GPU 비활성화
tf.config.set_visible_devices([], 'GPU')

# 흑돌 : False, 백돌 : True
FIRST_PLAY = True
# 베스트 플레이어 모델 로드
model = load_model("/home/gyeonheal/catkin_ws/src/ur_python/src/model/best.h5")

# 게임 UI 정의
class Gomoku():
    # 초기화
    def __init__(self, model=None):
        rospy.init_node('Gomoku_Alphago', anonymous=True)

        # 게임 상태 생성
        self.state = State()

        # PV MCTS를 활용한 행동 선택을 따르는 함수 생성
        self.next_action = pv_mcts_action(model, 0.0)

        self.last_position = 999
        self.fin_flag = 0
        # Publisher
        self.pub_ = rospy.Publisher('Ai_Stone_Coord', Int32MultiArray, queue_size=10)

        # Subscriber
        self.sub_ = rospy.Subscriber("User_Stone_Coord", Int32MultiArray, self.callbackfunc)

        self.user_x = -1
        self.user_y = -1
        self.user_x_prev = -2
        self.user_y_prev = -2

        print("")
        print("")
        print("**************Game has started**************")
        print("")
        print("")
        if FIRST_PLAY:
            # 사람의 턴으로 시작
            self.turn_of_human()
        else:
            # AI의 턴으로 시작
            self.turn_of_ai()

    def callbackfunc(self, data):
        self.user_x = data.data[0]
        self.user_y = data.data[1]

    # 사람의 턴
    def turn_of_human(self):

        # if self.state.is_done():
        #     print("is done Error")
        #     if FIRST_PLAY:
        #         self.state = State()
        #         self.on_draw()

        #     else:
        #         self.state = State()
        #         self.turn_of_ai()
        #         self.on_draw()
        #     return
        print("------------User's Turn------------")
        print("")
        print("--------------Pending--------------")
        print("")
        print("--------------Pending--------------")
        print("")
        print("--------------Pending--------------")
        if FIRST_PLAY:
            # 선수가 아닌 경우
            if not self.state.is_first_player():
                print("Line 75 Error")
                return
        else:
            if self.state.is_first_player():
                print("Line 78 Error")
                return

        # 범위 밖의 입력 방지
        while(self.user_x < 0 or 14 < self.user_x or self.user_y < 0 or 14 < self.user_y or(self.user_x_prev == self.user_x and self.user_y_prev == self.user_y)):
            continue
        
        print(f"Human Placed: {self.user_x}, {self.user_y}")

        self.user_x_prev = self.user_x 
        self.user_y_prev = self.user_y            
        action = self.user_x + self.user_y * 15

        # 합법적인 수가 아닌 경우
        if action not in self.state.legal_actions():
            print("This is not legal action. Game End")
            return

        # 상태 갱신 (사람의 돌을 놓은 후)
        self.state.pieces[action] = 1  # 사람은 1로 표시
        self.on_draw()
        if self.fin_flag:
            Ai_Coord= Int32MultiArray()
            Ai_Coord.data = [int(100),int(100)]
            rospy.loginfo(Ai_Coord)    
            self.pub_.publish(Ai_Coord)
            return

        # 게임 종료 여부 확인
        if self.state.is_done():
            print("게임이 종료되었습니다.")
            self.fin_flag = 1
        # AI 차례로 전환
        self.turn_of_ai()

    # AI의 턴
    def turn_of_ai(self):
        # AI의 선택
        print("--------------AI Turn------------")
        print("")
        print("--------------Pending--------------")
        print("")
        print("--------------Pending--------------")
        print("")
        print("--------------Pending--------------")
        action = self.next_action(self.state)

        if action is None:
            print("AI가 돌을 놓을 수 없습니다.")
            return

        self.state.enemy_pieces[action] = 1  # AI는 1로 표시 (예시로 흑돌로 설정)

        Ai_x = action % 15
        Ai_y = action // 15

        Ai_Coord = Int32MultiArray()

        self.on_draw()
        

        Ai_Coord.data = [int(Ai_x), int(Ai_y)]  # Assign x and y as a list
        rospy.loginfo(Ai_Coord)    
        self.pub_.publish(Ai_Coord)
        if self.fin_flag:
            Ai_Coord= Int32MultiArray()
            Ai_Coord.data = [int(-100),int(-100)]
            rospy.loginfo(Ai_Coord)    
            self.pub_.publish(Ai_Coord)
            return
        if self.state.is_done():
            print("게임이 종료되었습니다.")
            self.fin_flag = 1
            
        
        print(f"AI는 ({Ai_x}, {Ai_y})에 두었습니다.")

        # # 사람의 턴으로 전환
        self.turn_of_human()
        # rospy.spin()

    # 그림 갱신 (게임판 그리기)
    def on_draw(self):
        for y in range(15):
            row = ''
            for x in range(15):
                idx = x + y * 15
                if self.state.pieces[idx] == 1:
                    row += ' X '  # 사람의 돌
                elif self.state.enemy_pieces[idx] == 1:
                    row += ' O '  # AI의 돌
                else:
                    row += ' . '
            print(row)

    def run(self):
        rospy.loginfo("Alphago is running.")
        rospy.spin()  # ROS 노드를 계속 실행하면서 콜백 대기

def main():
    try:
        game = Gomoku(model=model)
        # game.run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


# 게임 실행 예시
if __name__ == "__main__":
    main()
