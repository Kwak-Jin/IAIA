# Troubleshooting

### Troubleshooting for the project

1. Gomoku game program sometimes [malfunctions](https://youtu.be/F8NwCMZV67k). These malfunction problem may lead to termination of the program using emergency stop. There are 2 main ideas tested in the project to prevent this situation
   
   1. `rospy.sleep(time)` is used as a temporary solution to the situation. This only helps if the robot has been operated too long.
   
   2. Giving more waypoints on the way to the destination. To do this, relative coordinates are defined as below
   
   ```python
   target_pose_abs_rpy = [0.00, 0.00, 0.00]         # Default rotation angle(0,0,0)  
   aaf_grip_pose_xyz = [-0.20, 0.00, 0.0]             # Relative Position after vacuum  (XYZ)  
   af_grip_pose_xyz = [0.00, 0.00, 0.233]             # Relative Position after vacuum  (XYZ)  
   grip_pose_xyz = [00.0, -0.00, -0.023]            # Relative Position vacuum        (XYZ)
   target_pose_abs_xyz = [-0.190, 0.3025, -0.305]    # Stone Placement position  
   ```
   
2. Checkers game have relative less range of movement than Gomoku games because there are no additional stones to place on the board so there are fewer waypoints/destination.
   ```python
   target_pose_abs_xyz_go = [0, 0, -0.043]          # Relative Position before vacuum (XYZ)  
   init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]  # waypoint between camera capture location and checkerboard
   target_pose_abs_rpy = [0.00, 0.00, 0.00]         # Default rotation angle(0,0,0)  
   ```
   
3. Communication between asynchronous processes may cause trouble within the process. To sync each steps(process), time-idling is used in the process by `while(not_changed)`. While not using time-idling, the robot may visit the previous stone coordinates on game board. To further change this inefficiency, multi-threading(like checkers game) can be suggested in a single or dual process.

<p align='center'><img src=".\image\flow table.png" alt="flow table" style="zoom:80%;" /> <br> Figure 1. Idle/Execute process </p>

4. Image processing without deep learning is used in the project for both gomoku and checkers game. This program is only available in the environment similar to NTH 115. If tested in other environment, the average `rbg` value may differ. To use this algorithm for general situation, adaptive thresholding method can be suggested in the program.

### Self-feedback

1. Time-management
2. Readability of code (함수화 부족/ 간결하지 못한 코드 정리)
3. C++ 오목 코드 실행 등 여러 시도를 하지 못함
4. rospy subscriber/publisher에서 발생하는 문제사항 해결하지 못함
