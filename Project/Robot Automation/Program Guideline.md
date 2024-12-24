# Project: Robot Automation

#### Author: 곽진, 한태건, 안견힐, 임순호

#### Date: 2024.12.18

#### 본 문서는 Industrial AI and Automation Project 2 보고서의 명령어 매뉴얼 부분을 한국어로 작성한 것이다.

 

본 문서는 Industrial AI and Automation의 Project 2 오목/체커 게임 실행을 위한 명령어 매뉴얼을 목적으로 작성되었다. 카메라, 오목판 등 하드웨어가 고정되어있는 상태에서 본 프로그램은 정상 작동한다.



## 파일 구조

로봇을 동작 시키기 위한 파일의 구조는 다음과 같다.

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
  |----|----|----|- user_idx.txt
  |----|----|----|- captured_images
  |----|----|----|----|- ...
```

위의 구조에 사용되었던 프로그램은 다음 [링크](https://github.com/Kwak-Jin/IAIA/tree/master/Project/Robot%20Automation/src)에서 다운로드 받을 수 있다.


본 프로그램을 수행하기 위해 필요한 환경은 다음과 같다.

## 소프트웨어 설정

- OS: Linux Ubuntu 20.04

- ROS: ROS Noetic

- Virtual Environment: Anaconda

- Additional packages required
  
  - Tensorflow >= 2.0.0
  
  - OpenCV >= 4.9.0
  
  - rospy 
  
  - numpy >= 2.0.0
  
  - moveit_ros_planning

- Additional Ubuntu utility programs
  
  - terminator
  - Visual Code

추가적인 소프트웨어 설정은 [링크](https://github.com/hyKangHGU/Industrial-AI-Automation_HGU/tree/main/tutorial/ubuntu)를 따른다.

## Terminal 명령어 모음

### 프로젝트 빌드 명령어

메세지 파일, 패키지 등을 불러오기 위하여 다음 명령로 catkin build를 시행한다.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

문제가 있을 경우에는 

1. 경로 확인(파이썬 경로)

2. Cmake파일의 `find_package()`에서 찾으려하는 패키지,메세지 등 존재 유무를 확인한다.
   
   

### 로봇 연결 명령어

로봇의 IP와 동기화할 수 있게 컴퓨터의 IP를 설정한 이후에 랜선을 연결하고 프로그램 상에서 IP 연결을 다음과 같이 수행한다.

```bash
  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2 
  roslaunch ur5e_rg2_moveit_config move_group.launch
```

### 체커 게임 실행

체커 게임의 경우 다음 명령어를 터미널 창에 수행한다.

```bash
conda activate py39
rosrun ur_python checkers.py
rosrun ur_python mc.py
```

### 오목 게임 실행

로봇과 연결되어있는 상태에서 다음과 같은 명령어를 통하여 오목 게임을 실행할 수 있다.

```bash
rosrun ur_python gomoku_image_capture.py
rosrun ur_python gomoku_image_processing.py
rosrun ur_python gomoku_move.py
```

Tensorflow가 로컬 파이썬 경로 내 있을 시, 가상환경 활성화 부분은 무시한다.

```bash
conda activate py39
rosrun ur_python gomoku.py
```

Terminator를 사용하여 `gomoku.py` 터미널 창 등을 디스플레이할 수 있다. 해당 프로그램은 현재 오목판 그리드를 Visualize한 상태이며, 로봇이 둘 위치, 유저가 둔 바둑돌 위치 등을 나타낸다.



### 에러 발생할 수 있는 부분

1. 프로그램 실행시에 카메라 포트 번호가 달라질 수 있기 때문에 OpenCV상 카메라 번호를 확인한다.

2. 로봇은 항상 정해져있는 지점 상에서 동작한다. 만약 로봇 끝단이 뒤를 향하거나, 지나치게 위/아래로 움직일 경우 E-stop을 통해 로봇을 긴급 정지한다. 
   
   
   
   
   
   
