# LAB: Pet Feeder Robot

Industrial AI & Automation 2024



**Name:**  

**Date:** 



---







## Introduction



This lab aims to implement an automated Pet Feeder Robot using ROS, with a pet classifier via camera.



![image](https://github.com/user-attachments/assets/68f36df1-60f4-46e9-8693-afc41855acf7)









## System Requirements

- Hardware
  - Robot: Indy 10
  - Camera: PC built-in camera (or USB camera)
- Software
  - Ubuntu 20.04
  - ROS Noetic
  - Pytorch (to use an Image-Net)







## Pet Feeder Robot System



![image](https://github.com/user-attachments/assets/94a97c2d-583c-49eb-878c-e027d015ae6f)







## Procedure





### Nodes Design

The main nodes for this robotic system are listed below. Briefly explain the role of each one.

- Camera
  - This node generate image data from camera sensor
  - publish `camera/image_raw`
- Image display
  - This node ~~
  - subscribe `~~`
- Pet Classifier
  - This node ~~
  - subscribe `~~`
  - subscribe `~~`
  - publish `~~`
- Pet feeder robot (=move_group_python_interface)
  - This node ~~
  - subscribe `~~`







### ROS Programming

- write source code for nodes in `catkin_ws/src/indy_driver/src`

  - `camera.py`
  - `image_display.py`
  - `pet_classifier.py`
  - `pet_feeder.py`
  - The source codes were provided in [UR5e-Execution](https://github.com/ykkimhgu/HGU_IAIA/blob/main/Tutorial/TU_ROS/tutorial/ur5e/ur5e-robot-execution.md), and needs to be appropriately modified.

  

- add message in `catkin_ws/src/indy_driver/msg`

  - `pet_info.msg`

  

- modify the file `CMakeList.txt` in `catkin_ws/src/indy_driver`



- if you want, you can make `.launch` file









### Simulation

Write the code here to run the simulation in the terminal

- terminal 1

```bash
roslaunch ~~ ~~~
```

- terminal 2

```bash
roslaunch ~~ ~~~
```









### Robot Execution

Write the code here to run the robot in the terminal

- terminal 1

```bash
roslaunch ~~ ~~~
```

- terminal 2

```bash
roslaunch ~~ ~~~
```









## Submission

- Lab Report (`.md`)
- Source code
- Demo video

