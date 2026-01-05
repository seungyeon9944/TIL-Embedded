## ROS2 실행 명령어

### `ros2 run`
- 패키지 내에서 단일 노드 실행
- 개별 노드 테스트하거나 디버깅할 때 많이 활용

- `ros2 run <패키지명> <실행파일명>`

### `ros2 launch`
- 여러 노드 동시에 실행
- 복잡한 시스템이나 다수의 노드 **동시에 실행**할 때 사용

- `ros2 launch <패키지명> <실행파일명>`

### `rqt_console`
- 디버깅할 때 에러 로그 메시지 탐색

- `ros2 run rqt_console rqt_console`

### Launching nodes
- `ros2 launch turtlesim multisim.launch.py`
- launch file 분석
- turtlesim control in terminal