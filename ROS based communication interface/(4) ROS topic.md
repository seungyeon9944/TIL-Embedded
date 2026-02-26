## ROS Topic
### Python Topic Subscription (구독)

1. `create_node`

```python
import rclpy as rp
from turtlesim.msg import Pose

rp.init()
test_node = rp.create_node('sub_test')
```

2. `callback` **함수** (토픽을 받을 때마다 어떤 일을 수행하게 하는 함수) 작성

```python
def callback(data):
	print("--->")
	print("/turtle1/pose : ", data)
	print("X : ", data.x)
	print("Y : ", data.y)
	print("Theta : ", data.theta) |
```

3. `create_subscription`

```python
test_node.create_subscription(Pose, '/turtle1/pose', callback, 10)
```

4. `spin_once` / `spin`

```python
rp.spin_once(test_node)
```

---

### Topic 횟수 제한해보기

```python
import rclpy as rp
from turtlesim.msg import Pose

rp.init()
test_node = rp.create_node('sub_test')

cnt = 0
def callback(data):
	global cnt
	cnt += 1
	print(">", cnt, " -> X : ", data.x, " Y : ", data.y)
	if cnt > 3:
		raise Exception("Subscription Stop")
```

---

### Python으로 Topic Publish (발행) 

1. 노드 만들기 

```python
import rclpy as rp
from geometry msgs.msg import Twist

rp.init()
test_node = rp.create_node('pub_test')
```

2. 데이터 타입 가져오기

```python
msg = Twist()
print(msg) # geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular = geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```

3. 변경

```python
msg.linear.x = 2.0
print(msg)
```

4. 실행

```python
pub = test_node.create_publisher(twist, '/turtle1/cmd_vel', 10)
pub.publish(msg)
```

5. (여러 번 보내기위해) 타이머를 이용한 콜백을 하나 만들고

```python
cnt = 0

def timer_callback():
	global cnt
	
	cnt += 1
	
	print(cnt)
	pub.publish(msg)
	
	if cnt > 3:
		raise Exception("Publisher Stop")
```

6. (여러 번 보내기 위해) 타이머를 선언해서 사용

```python
timer_period = 0.1
timer = test_node.create_timer(timer_period, timer_callback)
rp.spin(test_node)
```

7. 노드 종료

```python
test_node.destroy_node()
```