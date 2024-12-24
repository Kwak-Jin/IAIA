# Robot Automation
**Course: Industrial AI and Automation 2024-2**   
**Advisor: Prof. YK Kim**   
Date: 2024-12-11   

**School of Mechanical and Control Engineering, Handong Global University**
1. Objective: Gomoku and Checker Game with AI
2. Collaborator: Jin Kwak, Taegeon Han, Soonho Lim, Gyeonheal An
3. OS: Linux Ubuntu 20.04
4. ROS: ROS noetic
5. Virtual Environment: Anaconda environment
6. Language: C++, Python(3.8)
7. Robot: Universal Robot, UR5e
   
The project is simply a Five Stone(오목) automatic system using robot operation.   

File structure   
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

**Package required**   
- tensorflow >= 2.0.0
- OpenCV >= 4.9.0
- numpy >= 2.0.0
   
Command for the operation    
**Before the robot starts**    
Use `chillguys.py` for camera port number detection.

``` bash
  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
  roslaunch ur5e_rg2_moveit_config move_group.launch
```
After robot starts, project execution command on bash   
``` bash
  rosrun ur_python gomoku_image_capture.py
  rosrun ur_python gomoku_image_processing.py
  rosrun ur_python gomoku_move.py
```
For Gomoku algorithm, Conda environment is used   
``` bash
conda activate py39
rosrun ur_python gomoku.py
```   
