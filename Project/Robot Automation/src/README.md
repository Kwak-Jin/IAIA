# Robot Automation
1. Objective: Gomoku and Checker Game with AI
2. Collaborator: Jin Kwak, Taegeon Han, Soonho Lim, Gyeonheal An
3. OS: Linux Ubuntu 20.04
4. Language: C++, Python
5. Course: Industrial AI and Automation 2024-2
6. Advisor: Prof. YK Kim
7. School of Mechanical and Control Engineering, Handong Global University

File structure
catkin_ws
|--build
|--devel
|--src
|    |--CMakeLists.txt
|    |--ur_python
|    |      |--msg
|    |      |--src
|    |      |   |--omoks.cpp
|    |      |   |--gomoku_image_processing.py
|    |      |   |--gomoku_image_capture.py
|    |      |   |--gomoku_move.py
|    |      |   |--move_group_python_interface.py
                
Before the robot starts
``` bash
  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
  roslaunch ur5e_rg2_moveit_config move_group.launch
```
After robot starts, project execution command on bash
``` bash
  rosrun ur_python gomoku_image_capture.py
  rosrun ur_python gomoku_image_processing.py
  rosrun ur_python gomoku_move.py
  rosrun ur_python gomoku
```

