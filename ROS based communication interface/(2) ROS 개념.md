## Turtlesim 🐢

### Install turtlesim
`sudo apt update`

`sudo apt install ros-humble-turtlesim`

`ros2 pkg executables turtlesim`

### Start turtlesim
`ros2 run turtlesim turtlesim_node`

### Use turtlesim
`ros2 run turtlesim turtle_teleop_key`

```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### Install rqt
`sudo apt update`

`sudo apt install '~nros-humble-rqt*'`

`rqt`

### Try the spawn service
- 새로운 서비스 생성
- Turtlesim 예제에서는 새로운 거북이 생성

### Try the set_pen service
- 선의 굵기, 색상 등 설정 방법

### Remapping
- turtle2를 제어하기 위해서는 두번째 teleop 노드가 필요
- `ros2 run turtlesim turtle_teleop_key -ros-args -remap turtle1/cmd_vel:=turtle2/cmd_vel`

### Close turtlesim
- turtlesim_node 터미널에서 ctrl + c
- turtle_teleop_key 터미널에서 q

---

## ROS 핵심개념

### Nodes
- Each node in ROS should be responsible for a single, modular purpose
- Each node can **send and receive data from other nodes via topics, services, actions or parameters**.

### Topics
Topics are one of the main ways in which **data is moved between nodes**

`ros2 topic list`하면 각 topic의 이름
- /turtle1/cmd_vel
- /turtle1/color_sensor
- /turtle1/pose
- /turtle2/cmd_vel
- /turtle2/color_sensor
- /turtle2/pose

`ros2 topic echo`하면 x, y, theta, linear_velocity, angular_velocity 구할 수 있음

### Services
- Services are another method of communication for nodes in the ROS graph
- Services are based on a call-and-response model versus the publisher-subscriber model of topics

### Parameters
A parameter is a configuration value of a node

---

## Simple publisher and subscriber

subscriber_node.py
```
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
        rclpy.init(args=args)
        node = HelloSubscriber()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

if __name__ == '__main__':
    main()
```

setup.py에서
```
entry_points = {
    'console_scripts':[
        ...,
        'subscriber_node = test_ros.subscriber_node:main',
    ],
},
```

등등 같은 방식으로 turtle_spawner 및 turtle_controller로 거북이 조종