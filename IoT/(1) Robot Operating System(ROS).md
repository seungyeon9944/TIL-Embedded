## ROS
ROS는 Robot Operating System의 약어
- 로봇을 개발하는데 있어 필수적인 라이브러리 제공
- Application 부분만 개발하면 되기 때문에 개발 시간, 비용 절약 효과

### 1. 노드
노드는 **ROS에서 최소 단위의 실행 프로세스**를 가리키는 용어, 즉 하나의 파이썬 스크립트는 하나의 노드

### 2. 메시지
노드에서 다른 노드로 정보를 전달하는 **단방향, 비동기식, 연속성** 통신

### 3. 패키지
**ROS 소프트웨어의 기본 단위**. 패키지는 노드, 라이브러리, 환경설정 파일들을 통합하는 최소의 빌드 단위이며, 배포 단위

![노드, 메시지, 패키지 다이어그램](https://i.imgur.com/mYlnSAi.png)

### 노드 명령어
```
$ ros2 node list # 실행되고 있는 노드 이름 출력
$ ros2 node info /talker # talker 노드의 통신 상태, 내용 출력
```

### 토픽(메시지) 명령어
```
$ ros2 topic list # publish 되고 있는 메시지 리스트 출력
$ ros2 topic echo /chatter # /chatter의 메시지 내용 출력
$ ros2 topic pub /test std_msgs/msg/Float64 "{data : 10}" # /test라는 메시지 publish
```

### ROS 장점
1. 노드 간 메시지 교환 방법으로 **복잡한 프로그램을 나눠** 공동 개발 용이

    ex) 인지 개발 - (인지 결과) → 판단 개발 - (판단 결과) → 제어 개발

2. 로봇 관련 **다양한 패키지** 제공 (센서 드라이버, 표준 메시지, Navigation Stack)

3. **강력한 시각화 도구** (RQT - rqt topic monitor, rqt graph, RVIZ) 혹은 메시지 기록, 재생 도구 (ROSBAG)