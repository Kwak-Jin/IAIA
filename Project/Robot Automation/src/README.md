# Robot Automation
1. Objective: Gomoku and Checker Game with AI
2. Collaborator: Jin Kwak, Taegeon Han, Soonho Lim, Gyeonheal An
3. OS: Linux Ubuntu 20.04
4. Language: C++, Python
5. Course: Industrial AI and Automation 2024-2
6. Advisor: Prof. YK Kim
7. School of Mechanical and Control Engineering, Handong Global University


``` bash
  roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
  roslaunch ur5e_rg2_moveit_config move_group.launch
  roscore
  rosrun ur_python publisher
  rosrun ur_python demo_move.py 
```
