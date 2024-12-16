# Project: Robot Automation

#### Authors:

- Jin Kwak(21900031)

- Gyeonheal An(21900416)

- Soonho Lim(21900610)

- Taegeon Han(21900793)

#### Date: 2024.12.16

#### [GitHub Link](https://github.com/Kwak-Jin/IAIA)

#### [Demo Video]()

## 1. Introduction

### 1.1 Overview

#### Background

The idea of physical operation of AlphaGo(Google DeepMind) in Gomoku and checker game is the motivation of this project. By this project, users may interact with an intelligent co-robot by playing games by themselves. In the project, UR5e(Universal Robot) is used with ROS-based robot control.

#### Objective

Implementation of automation process using a co-robot has been an important task in manufacturing industry or even in everyday life.

To collaborate with robot, it is important to achieve several tasks as below:

- Intelligent decision-making using AI algorithms
- Real-time feedback of human interaction

#### Goal

- Win rate of both gomoku and checker algorithm: 80% 
- Classification of stones accuracy: over 95%
- Position detection of stones accuracy: over 95%

#### Expected outcome

- 

### 1.2. Requirement

#### Software

- OS: Linux Ubuntu 20.04
- ROS: ROS Noetic
- Virtual Environment: Anaconda
- Additional packages required

  - Tensorflow

  - OpenCV

  - rospy

  - numpy

- Additional Ubuntu utility programs
  - terminator
  - Visual Code


To build overall software setting, follow the [link](https://github.com/hyKangHGU/Industrial-AI-Automation_HGU/tree/main/tutorial/ubuntu) for extra guideline.

#### Hardware

- Co-Robot: UR5e
- Gripper: Vacuum gripper
- Camera: ODROID USB-CAM 720P
- Camera holder

<p align='center'><img src=".\image\camera_holder.png" alt="camera_holder" style="zoom:65%;" />Figure Camera holder</p>

Camera holder is designed and 3D printed. Camera holder is attached to the robot arm as figure 번호 goes here

실제 Figure goes here

- Stone tray

<p align='center'><img src=".\image\stone_tray.png" alt="stone_tray" style="zoom:70%;" />Figure Stone tray</p>

For Gomoku game, the robot should be able to reload a new stone. To regularize the robot action, stone tray is designed and placed near UR5e robot.

- Game board



- Optic table

<p align='center'><img src=".\image\optic_table.png" alt="optic_table" style="zoom:60%;" />Figure Optic table</p>

The table is designed for UR5e robot, game board, stone tray to be fixed in right position. 

## 2. Problem

### 2.1  Problem for both Gomoku and Checker

- Stone position on game board  
- Detection of user's stone position
- Calculation of robot's stone coordinates

### 2.2. Problem description for Gomoku

-  Reloading of stone on the stone tray

### 2.3. Problem description for Checker game

- Change of stone's position within the board
- Update removed stone/moved stone.

## 3. Algorithm

### 3.1. Logic Design

As both checker game, gomoku game follows steps(perception decision-making control), the system's following flowchart is drawn as below:

<p align='center'><img src=".\image\Gomoku.drawio.png" alt="Gomoku.drawio" style="zoom:120%;" /> Figure Flowchart</p>

Flowchart is divided into processes and these processes can be drawn as RQT graph as below:

<p align='center'><img src=".\image\rqt_graph.png" alt="Gomoku.drawio" style="zoom:120%;" /> Figure RQT graph</p>

Each block except `human interaction` represents each python execution files. Each arrow represents message or information. 

1. Gomoku/Checker algorithm: `checkers.py` or `gomoku.py`
2. UR5e: `mc.py` for checker game or `gomoku_move.py`
3. Camera:  `gomoku_image_capture.py` or `mc.py`
4. Image Processing and Display: `gomoku_image_processing.py` or `mc.py`

#### 3.1.1. Image Processing

For better image processing, there is a green padding around the game board(Figure 숫자 goes here) in order to separate game board with the environment.

<p align='center'><img src=".\image\checker_real.png" alt="checker_real" style="zoom:80%;" />Figure Game board</p>

Step 1. Image Capture:

Step 2. Detect the game board:

Step 3. Warp perspective:

Step 4. Find black stones on the game board:

Step 5. Exclude existed stones:

Step 6. Find the newly placed stone:

Step 7. Publish the newly placed stone coordinates:

#### 3.1.2. Game algorithm

#### 3.1.2a. Gomoku algorithm



####  3.1.2b. Checker algorithm



#### 3.1.3. Robot manipulation

There are 2 ways to control UR5-e robot:

<p align='center'><img src=".\image\kinematic.png" alt="kinematic" style="zoom:80%;" /> Figure Robot Kinematics</p>

1. Adjusting robot's joint angles using a method `go_to_joint_abs(joint_angles)` in `MoveGroupPythonInterface` class.
   - Easy to control a robot movement.
   - The displacement of an end-effector is not known
   - Forward Kinematics
2. Adjusting robot's final displacement using a method `go_to_pose_rel(xyz_position, end_effector_angle)` in `MoveGroupPythonInterface` class.
   - Easy to move a robot by changing end effector's relative displacement
   - The angle of an end-effector should be calculated by numerically(Jacobian Matrix)
   - Inverse Kinematics
   - Sometimes, this control method may have multiple solutions of joint angles and this may cause trouble [as shown in troubleshooting](#Troubleshooting)

End-effector should always face down while picking or placing a stone. Therefore, end-effector's angle is always set as below. 

```python
grip_pose_rpy = [0.00, 0.00, 0.00]
```



#### 3.1.3a. Robot manipulation for Gomoku game

To move the end-effector to the stone tray and the game board with uniform movement,

#### 3.1.3b. Robot manipulation for Checker game



### 3.2. Code

The **file structure** is the following:

```
catkin_ws
  |- build
  |- devel
  |- src
  |----|- CMakeLists.txt
  |----|- ur_python
  |----|----|- msg
  |----|----|----|- object_info.msg
  |----|----|----|- robot_state.msg
  |----|----|----|- capture_flag.msg
  |----|----|----|- capture_finished_flag.msg
  |----|----|----|- ...
  |----|----|- src
  |----|----|----|- dual_network.py
  |----|----|----|- gomoku.py
  |----|----|----|- human_play.py
  |----|----|----|- omok.py
  |----|----|----|- pv_mcts.py
  |----|----|----|- checkers.py
  |----|----|----|- move_coord.py
  |----|----|----|- move_check.py
  |----|----|----|- mc.py
  |----|----|----|- omoks.cpp
  |----|----|----|- gomoku.py
  |----|----|----|- gomoku_image_processing.py
  |----|----|----|- gomoku_image_capture.py
  |----|----|----|- gomoku_move.py
  |----|----|----|- move_group_python_interface.py
```

To activate the system, the robot should be connected to a computer.

**Configuration for the robot**

```bash
  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2 
  roslaunch ur5e_rg2_moveit_config move_group.launch
```

1. Option: Checker Program

```bash
conda activate py39
rosrun ur_python checkers.py
rosrun ur_python mc.py
```

2. Option: Gomoku Program

```bash
rosrun ur_python gomoku_image_capture.py
rosrun ur_python gomoku_image_processing.py
rosrun ur_python gomoku_move.py
```

In the following command, gomoku algorithm process is activated in virtual environment. (If there is a tensorflow package in the local python, conda activation is not required)

```bash
conda activate py39
rosrun ur_python gomoku.py
```

This individual terminal commands are inconvenient thus, bash shell script(`.sh`) executed in this project.

```sh
```



## 4. Results and Demo

### 4.1. Results



[Demo Video]()

### 4.2. Discussion



## 5. Conclusion



### Further works

For further improvements of the collaborative robot, robot must behavior in uniform movements. 

## 6. Reference

### Appendix

[1] https://deepmind.google/research/breakthroughs/alphago/

[2] https://github.com/hyKangHGU/Industrial-AI-Automation_HGU

[3] https://github.com/Jpub/AlphaZero

###  Troubleshooting

1. Gomoku game program sometimes malfunctions. 
2. Communication between asynchronous processes may cause trouble within the process. To sync each steps(process), time-idling is used in the process by `while(not_changed)`. While not using time-idling, the robot may visit the previous stone coordinates on game board.

<p align='center'><img src=".\image\flow table.png" alt="flow table" style="zoom:75%;" />Figure Idle/Execute process </p>

### Code Appendix

`CMakeLists.txt`

```cmake
```

`Gomoku_move.py`

```python
```

`Gomoku.py`

```python
```

`Gomoku_image_capture.py`

```python
```

`Gomoku_image_processing.py`

```python
```

`mc.py`

```python
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
```

`checkers.py`

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*-


#//BUSHRA ASHFAQUE CS-19011//

from copy import deepcopy
import time
import math
import cv2
from move_coord import image2coordinate

import os
import rospy
from std_msgs.msg import String


ansi_black = "\u001b[30m"
ansi_red = "\u001b[31m"
ansi_green = "\u001b[32m"
ansi_yellow = "\u001b[33m"
ansi_blue = "\u001b[34m"
ansi_magenta = "\u001b[35m"
ansi_cyan = "\u001b[36m"
ansi_white = "\u001b[37m"
ansi_reset = "\u001b[0m"
com = 0


class Node:
    def __init__(self, board, move=None, parent=None, value=None):
        self.board = board
        self.value = value
        self.move = move
        self.parent = parent

    def get_children(self, maximizing_player, mandatory_jumping):
        current_state = deepcopy(self.board)
        available_moves = []
        children_states = []
        big_letter = ""
        queen_row = 0
        if maximizing_player is True:
            available_moves = Checkers.find_available_moves(current_state, mandatory_jumping)
            big_letter = "C"
            queen_row = 7
        else:
            available_moves = Checkers.find_player_available_moves(current_state, mandatory_jumping)
            big_letter = "B"
            queen_row = 0
        for i in range(len(available_moves)):
            old_i = available_moves[i][0]
            old_j = available_moves[i][1]
            new_i = available_moves[i][2]
            new_j = available_moves[i][3]
            state = deepcopy(current_state)
            Checkers.make_a_move(state, old_i, old_j, new_i, new_j, big_letter, queen_row)
            children_states.append(Node(state, [old_i, old_j, new_i, new_j]))
        return children_states

    def set_value(self, value):
        self.value = value

    def get_value(self):
        return self.value

    def get_board(self):
        return self.board

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent


class Checkers:
    def __init__(self):
        # # ROS Publisher 초기화
        rospy.init_node('checkers_move_publisher', anonymous=True)
        self.task_status = False
        self.move_pub = rospy.Publisher('/checkers/move', String, queue_size=10)
        self.read_robot_state = rospy.Subscriber('/task_status', String, self.callback)

        self.matrix = [[], [], [], [], [], [], [], []]
        self.current_turn = True
        self.computer_pieces = 12
        self.player_pieces = 12
        self.available_moves = []
        self.mandatory_jumping = False

        self.firststart = True

        for row in self.matrix:
            for i in range(8):
                row.append("---")
        self.position_computer()
        self.position_player()

    def position_computer(self):
        for i in range(3):
            for j in range(8):
                if (i + j) % 2 == 1:
                    self.matrix[i][j] = ("c" + str(i) + str(j))

    def position_player(self):
        for i in range(5, 8, 1):
            for j in range(8):
                if (i + j) % 2 == 1:
                    self.matrix[i][j] = ("b" + str(i) + str(j))

    def cap_video(self):
        while True:
            # video capture            
            self.cap = cv2.VideoCapture(2)
            self.success, self.frame = self.cap.read()
            if self.success:
                print("successfully captured image")
                self.cap.release()
                break

    def print_matrix(self):
        i = 0
        print()
        for row in self.matrix:
            print(i, end="  |")
            i += 1
            for elem in row:
                print(elem, end=" ")
            print()
        print()
        for j in range(8):
            if j == 0:
                j = "     0"
            print(j, end="   ")
        print("\n")


    def delete_file(file_path):
        try:
            if os.path.exists(file_path):
                os.remove(file_path)
        except Exception as e:
            print(f"Error occur when erasing file: {e}")

    #----------------------------------------- get message from the robot move file (mc.py) ----------------------------------#
    def callback(self, msg):
        self.task_status = True
        print(f"Message received: {msg.data}")
        # 받은 후 처리하고, 다시 False로 초기화
        time.sleep(1)  # 잠시 대기 (필요한 처리 추가 가능)
        self.task_status = False

    # def check_status(self):
    #     if self.task_status:
    #         print("Task is currently in progress...")
    #     else:
    #         print("No task in progress.")

    #------------------------------------------------------------------------------------------------------------------------#

    #----------------------------------- capture 된 돌 좌표 없애기 --------------------------------------------------#
    def remove_coordinate_from_file(self, file_path, target_coordinate):

        target_str = f"({target_coordinate[0]}, {target_coordinate[1]})"  # 좌표를 문자열로 변환
        lines_to_keep = []  # 삭제되지 않은 라인을 저장할 리스트

        # 파일을 읽고, 각 라인을 확인하여 좌표가 포함되어 있지 않으면 lines_to_keep에 추가
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if target_str not in line:
                    lines_to_keep.append(line)

        # 수정된 내용을 파일에 다시 저장
        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(lines_to_keep)
        print(f"좌표 {target_coordinate}가 파일에서 삭제되었습니다.")

    def check_coordinate_and_remove(self, file_path, target_coordinate):

        target_str = f"({target_coordinate[0]}, {target_coordinate[1]})"  # 좌표를 문자열로 변환

        # 파일에서 좌표가 포함되어 있는지 확인
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if target_str in line:
                    self.remove_coordinate_from_file(file_path, target_coordinate)
                    return True  # 좌표가 발견되면 True 반환

        return False  # 좌표가 발견되지 않으면 False 반환
    #----------------------------------------------------------------------------------------------------------------#

    def get_player_input(self):
        
        available_moves = Checkers.find_player_available_moves(self.matrix, self.mandatory_jumping)
        if len(available_moves) == 0:
            if self.computer_pieces > self.player_pieces:
                print(
                    ansi_red + "You have no moves left, and you have fewer pieces than the computer.YOU LOSE!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            else:
                print(ansi_yellow + "You have no available moves.\nGAME ENDED!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
        self.player_pieces = 0
        self.computer_pieces = 0
        while True:
    # ------------------ Modified image to coordinate input from user --------------------- #
            # wait until the move "press Enter"

            while True:

                # place = input("press enter after your stone placement")

                # if not place:
                #     print("stone place completed")
                #     break

                if self.firststart:
                    print("Its your turn make a move")
                    t = 8
                    print("Waiting for 8 seconds...")
                    for i in range(t, 0, -1):
                        print(f"{i}s left", end = "\r")
                        time.sleep(1)  # 5초 기다림

                    self.firststart = False
                    break


                if self.task_status:
                    t = 8
                    print("Waiting for 8 seconds...")
                    for i in range(t, 0, -1):
                        print(f"{i}s left", end = "\r")
                        time.sleep(1)  # 5초 기다림
                    # w = 2
                    # for i in range(w, 0, -1):
                    #     print(f"Your turn is up put your hand away from the board {i} sec", "\r")
                    #     time.sleep(1)
                    break

            print("Capturing Video")
            self.cap_video()

    #-------------------------- Figuring out the captured stone of the user & erasing form the coordinate text-----------------------------#
            x = 0
            y = 0
            if com:
                print (com[0],com[1],com[2], com[3])
                y1 = com[0]
                y2 = com[2]
                x1 = com[1]
                x2 = com[3]

                if abs(x2 - x1) >= 2:
                    if (x2-x1) > 0 and (y2-y1) > 0:
                        x = x2 -1
                        y = y2 -1
                    elif (x2-x1) > 0 and (y2-y1) < 0:
                        x = x2 -1
                        y = y2 +1
                    elif (x2-x1) < 0 and (y2-y1) > 0:
                        x = x2 +1
                        y = y2 -1
                    elif (x2-x1)< 0 and (y2-y1) < 0:
                        x = x2 +1
                        y = y2 +1

                    target_coordinate = (y, x)
                    self.check_coordinate_and_remove("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt", target_coordinate)
    #--------------------------------------------------------------------------------------------------------------------------------------#

    #-------------------------------------------------- image to coordinate ---------------------------------------------------------------#
            coord1_set, coord2_set = image2coordinate(self.frame)

            # coord1과 coord2가 비어있는지 확인
            if not coord1_set or not coord2_set:
                print("Error: coord1 or coord2 is empty!")
                
            else:
                # set에서 튜플을 추출한 뒤 문자열로 변환
                coord1 = f"{next(iter(coord1_set))[0]},{next(iter(coord1_set))[1]}"
                coord2 = f"{next(iter(coord2_set))[0]},{next(iter(coord2_set))[1]}"

                # 결과 출력
                print(f"Mock input coord1: {coord1}")
                print(f"Mock input coord2: {coord2}")
    # --------------------------------------------------------------------------------------------------------------------------------------#
                
            if coord1 == "":
                print(ansi_cyan + "Game ended!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif coord1 == "s":
                print(ansi_cyan + "You surrendered.\nCoward." + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()

            if coord2 == "":
                print(ansi_cyan + "Game ended!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif coord2 == "s":
                print(ansi_cyan + "You surrendered.\nCoward." + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            old = coord1.split(",")
            new = coord2.split(",")

            if len(old) != 2 or len(new) != 2:
                print(ansi_red + "Ilegal input" + ansi_reset)
            else:
                old_i = old[0]
                old_j = old[1]
                new_i = new[0]
                new_j = new[1]
                if not old_i.isdigit() or not old_j.isdigit() or not new_i.isdigit() or not new_j.isdigit():
                    print(ansi_red + "Ilegal input" + ansi_reset)
                else:
                    move = [int(old_i), int(old_j), int(new_i), int(new_j)]
                    if move not in available_moves:
                        print(ansi_red + "Ilegal move!" + ansi_reset)
                    else:
                        Checkers.make_a_move(self.matrix, int(old_i), int(old_j), int(new_i), int(new_j), "B", 0)
                        for m in range(8):
                            for n in range(8):
                                if self.matrix[m][n][0] == "c" or self.matrix[m][n][0] == "C":
                                    self.computer_pieces += 1
                                elif self.matrix[m][n][0] == "b" or self.matrix[m][n][0] == "B":
                                    self.player_pieces += 1
                        print(self.matrix)

                        break

    @staticmethod
    def find_available_moves(board, mandatory_jumping):
        available_moves = []
        available_jumps = []
        for m in range(8):
            for n in range(8):
                if board[m][n][0] == "c":
                    if Checkers.check_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
                elif board[m][n][0] == "C":
                    if Checkers.check_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                    if Checkers.check_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
        if mandatory_jumping is False:
            available_jumps.extend(available_moves)
            return available_jumps
        elif mandatory_jumping is True:
            if len(available_jumps) == 0:
                return available_moves
            else:
                return available_jumps

    @staticmethod
    def check_jumps(board, old_i, old_j, via_i, via_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[via_i][via_j] == "---":
            return False
        if board[via_i][via_j][0] == "C" or board[via_i][via_j][0] == "c":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[old_i][old_j][0] == "b" or board[old_i][old_j][0] == "B":
            return False
        return True

    @staticmethod
    def check_moves(board, old_i, old_j, new_i, new_j):

        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j][0] == "b" or board[old_i][old_j][0] == "B":
            return False
        if board[new_i][new_j] == "---":
            return True

    @staticmethod
    def calculate_heuristics(board):
        result = 0
        mine = 0
        opp = 0
        for i in range(8):
            for j in range(8):
                if board[i][j][0] == "c" or board[i][j][0] == "C":
                    mine += 1

                    if board[i][j][0] == "c":
                        result += 5
                    if board[i][j][0] == "C":
                        result += 10
                    if i == 0 or j == 0 or i == 7 or j == 7:
                        result += 7
                    if i + 1 > 7 or j - 1 < 0 or i - 1 < 0 or j + 1 > 7:
                        continue
                    if (board[i + 1][j - 1][0] == "b" or board[i + 1][j - 1][0] == "B") and board[i - 1][
                        j + 1] == "---":
                        result -= 3
                    if (board[i + 1][j + 1][0] == "b" or board[i + 1][j + 1] == "B") and board[i - 1][j - 1] == "---":
                        result -= 3
                    if board[i - 1][j - 1][0] == "B" and board[i + 1][j + 1] == "---":
                        result -= 3

                    if board[i - 1][j + 1][0] == "B" and board[i + 1][j - 1] == "---":
                        result -= 3
                    if i + 2 > 7 or i - 2 < 0:
                        continue
                    if (board[i + 1][j - 1][0] == "B" or board[i + 1][j - 1][0] == "b") and board[i + 2][
                        j - 2] == "---":
                        result += 6
                    if i + 2 > 7 or j + 2 > 7:
                        continue
                    if (board[i + 1][j + 1][0] == "B" or board[i + 1][j + 1][0] == "b") and board[i + 2][
                        j + 2] == "---":
                        result += 6

                elif board[i][j][0] == "b" or board[i][j][0] == "B":
                    opp += 1

        return result + (mine - opp) * 1000

    @staticmethod
    def find_player_available_moves(board, mandatory_jumping):
        available_moves = []
        available_jumps = []
        for m in range(8):
            for n in range(8):
                if board[m][n][0] == "b":
                    if Checkers.check_player_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_player_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                elif board[m][n][0] == "B":
                    if Checkers.check_player_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_player_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                    if Checkers.check_player_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_player_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_player_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
        if mandatory_jumping is False:
            available_jumps.extend(available_moves)
            return available_jumps
        elif mandatory_jumping is True:
            if len(available_jumps) == 0:
                return available_moves
            else:
                return available_jumps

    @staticmethod
    def check_player_moves(board, old_i, old_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j][0] == "c" or board[old_i][old_j][0] == "C":
            return False
        if board[new_i][new_j] == "---":
            return True

    @staticmethod
    def check_player_jumps(board, old_i, old_j, via_i, via_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[via_i][via_j] == "---":
            return False
        if board[via_i][via_j][0] == "B" or board[via_i][via_j][0] == "b":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[old_i][old_j][0] == "c" or board[old_i][old_j][0] == "C":
            return False
        return True

    def evaluate_states(self):
        global com 

        t1 = time.time()
        current_state = Node(deepcopy(self.matrix))

        first_computer_moves = current_state.get_children(True, self.mandatory_jumping)
        if len(first_computer_moves) == 0:
            if self.player_pieces > self.computer_pieces:
                print(
                    ansi_yellow + "Computer has no available moves left, and you have more pieces left.\nYOU WIN!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            else:
                print(ansi_yellow + "Computer has no available moves left.\nGAME ENDED!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
        dict = {}
        for i in range(len(first_computer_moves)):
            child = first_computer_moves[i]
            value = Checkers.minimax(child.get_board(), 4, -math.inf, math.inf, False, self.mandatory_jumping)
            dict[value] = child

        if len(dict.keys()) == 0:
            print(ansi_green + "Computer has cornered itself.\nYOU WIN!" + ansi_reset)
            self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
            exit()
        new_board = dict[max(dict)].get_board()
        move = dict[max(dict)].move
        self.matrix = new_board
        t2 = time.time()
        diff = t2 - t1

        # ROS 메시지로 move를 Publish
        move_msg = f"{move[0]},{move[1]}->{move[2]},{move[3]}"
        self.move_pub.publish(move_msg)  # 메시지 발행
        print(f"Published move: {move_msg}")

        # save computer coordinate 
        com = move

        print("Computer has moved (" + str(move[0]) + "," + str(move[1]) + ") to (" + str(move[2]) + "," + str(
            move[3]) + ").")
        print("It took him " + str(diff) + " seconds.")

    @staticmethod
    def minimax(board, depth, alpha, beta, maximizing_player, mandatory_jumping):
        if depth == 0:
            return Checkers.calculate_heuristics(board)
        current_state = Node(deepcopy(board))
        if maximizing_player is True:
            max_eval = -math.inf
            for child in current_state.get_children(True, mandatory_jumping):
                ev = Checkers.minimax(child.get_board(), depth - 1, alpha, beta, False, mandatory_jumping)
                max_eval = max(max_eval, ev)
                alpha = max(alpha, ev)
                if beta <= alpha:
                    break
            current_state.set_value(max_eval)
            return max_eval
        else:
            min_eval = math.inf
            for child in current_state.get_children(False, mandatory_jumping):
                ev = Checkers.minimax(child.get_board(), depth - 1, alpha, beta, True, mandatory_jumping)
                min_eval = min(min_eval, ev)
                beta = min(beta, ev)
                if beta <= alpha:
                    break
            current_state.set_value(min_eval)
            return min_eval

    @staticmethod
    def make_a_move(board, old_i, old_j, new_i, new_j, big_letter, queen_row):
        letter = board[old_i][old_j][0]
        i_difference = old_i - new_i
        j_difference = old_j - new_j
        if i_difference == -2 and j_difference == 2:
            board[old_i + 1][old_j - 1] = "---"

        elif i_difference == 2 and j_difference == 2:
            board[old_i - 1][old_j - 1] = "---"

        elif i_difference == 2 and j_difference == -2:
            board[old_i - 1][old_j + 1] = "---"

        elif i_difference == -2 and j_difference == -2:
            board[old_i + 1][old_j + 1] = "---"

        if new_i == queen_row:
            letter = big_letter
        board[old_i][old_j] = "---"
        board[new_i][new_j] = letter + str(new_i) + str(new_j)

    def play(self):
        print(ansi_cyan + "##### WELCOME TO CHECKERS ####" + ansi_reset)
        print("\nSome basic rules:")
        print("1.You enter the coordinates in the form i,j.")
        print("2.You can quit the game at any time by pressing enter.")
        print("3.You can surrender at any time by pressing 's'.")
        print("Now that you've familiarized yourself with the rules, enjoy!")
        while True:
            answer = input("\nFirst, we need to know, is jumping mandatory?[Y/n]: ")
            # answer = "n"
            if answer == "Y":
                self.mandatory_jumping = True
                break
            elif answer == "n":
                self.mandatory_jumping = False
                break
            else:
                print(ansi_red + "Invalid option!" + ansi_reset)
        while True:
            self.print_matrix()
            if self.current_turn is True:
                print(ansi_cyan + "\nPlayer's turn." + ansi_reset)
                self.get_player_input()
            else:
                print(ansi_cyan + "Computer's turn." + ansi_reset)
                print("Thinking...")
                self.evaluate_states()
            if self.player_pieces == 0:
                self.print_matrix()
                print(ansi_red + "You have no pieces left.\nYOU LOSE!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif self.computer_pieces == 0:
                self.print_matrix()
                print(ansi_green + "Computer has no pieces left.\nYOU WIN!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            self.current_turn = not self.current_turn


if __name__ == '__main__':
    checkers = Checkers()
    checkers.play()
```
