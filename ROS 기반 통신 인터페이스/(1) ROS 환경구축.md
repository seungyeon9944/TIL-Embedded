## Robot Operating System - ROS humble 설치

### ROS 프로젝트 구현
**워크스페이스 (ws)** > **패키지 (pkg)** > **노드 (Node)**

- 패키지 빌드할 때는 워크스페이스 내부
- 패키지는 워크스페이스 내부의 src 폴더 내에서 

### 패키지 생성
`code ~/.bashrc`를 실행해서 제일 하단에 `source /opt/ros/humble/setup.bash` 코드 추가

터미널 창에서 `source ~/.bashrc` 실행해서 bashrc가 업데이트되었다는걸 알려줌

`ros2 pkg create --build-type ament_python test_ros`

### 노드 생성

publisher_node.py
```
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello publisher')

        self.publisher = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS2: {self.count}'
        self.publisher_publish(msg)
        self.get_logger().info(f'publishing: "{msg.data}"')
        self.count += 1


    def main(args=None):
        rclpy.init(args=args)
        node = HelloPublisher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
```

setup.py에서
```
...
entry_points={
    'console_scripts': [
        'publisher_node = test_ros.publisher_node:main',
    ],
}
```

.bashrc에 `source ~/ros2_ws/install/local_setup.bash` 추가