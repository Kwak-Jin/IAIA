# ROS

Author: Jin Kwak

**주의할 점**

1. catkin_make할 시 ROS python path를 참조해야함.
2. 동작은 콘다 환경/ ROS환경 상관 없다.
3. catkin_make 이후, 초기화를 해야한다.
4. act_ros()
5. 저 같은 경우 이미 WSL 환경에 콘다(default) 구축까지 완료된 상태라 생긴 문제가 많았습니다.
6. xml 파일에 의존성 패키지 등 적어서 catkin으로 빌드해야함.
7. launch를 사용하면 각 노드 동시 실행(roscore 없이 한다한거같다.) 

`roscore` : ROS start

`rosrun` : Node

`rqt_graph` : Node Message Check