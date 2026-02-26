## ROS service client

1. `client_test` 노드 만들기

```python
import rclpy as rp
from turtlesim.srv import TeleportAbsolute

rp.init()
test_node = rp.create_node('client_test')
```

2. `create_client` 사용

```python
service_name = '/turtle1/teleport_absolute'
cli = test_node.create_client(TeleportAbsolute, service_name)
```

3. 서비스 정의를 사용할 준비

```python
req = TeleportAbsolute.Request()
req # turtlesim.srv.TeleportAbsolute_Request(x=0.0, y=0.0, theta=0.0)
```

4. 서비스 call

```python
req.x = 3.

cli.call_async(req)
rp.spin_once(test_node)
```

5. `wait_for_service`

```python
req.y = float(9)

# 서비스가 준비중일 때까지 기다려라
while not cli.wait for service(timeout_sec=1.0):
	print("Waiting for service")
	
cli.call_async(req)
rp.spin_once(test_node)
```

6. 서비스가 실행되는 결과 받아오기

```python
req.x = float(9)

future = cli.call_async(req)

while not future.done():
	rp.spin_once(test_node)
	print(future.done(), future.result())
```