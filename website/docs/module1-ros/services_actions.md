# ROS 2 Services and Actions

Building upon nodes and topics, ROS 2 also provides higher-level communication mechanisms for specific interaction patterns: Services and Actions. These are crucial for robust and structured communication in a robotics system.

## ROS 2 Services: Request-Response Communication

Services in ROS 2 enable a synchronous, request-response communication pattern between nodes. This is ideal for operations where a client node needs a specific computation or data from a server node and can afford to wait for the response.

Key characteristics of ROS 2 Services:
- **Synchronous**: The client waits for the server's response.
- **Client-Server Model**: One node acts as a service server, providing a service, and another acts as a client, requesting it.
- **Service Definition**: Services have a `.srv` file defining the request and response message structure.

### Practical Example: Simple Adder Service

Consider a service that takes two integers as a request and returns their sum as a response.

#### Service Definition (`AddTwoInts.srv`)

```
int64 a
int64 b
---
int64 sum
```

#### Service Server Node (`add_two_ints_server.py`)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client Node (`add_two_ints_client.py`)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run <package_name> add_two_ints_client A B')
        return

    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    add_two_ints_client.get_logger().info(
        f'Result of add_two_ints: for {add_two_ints_client.req.a} + {add_two_ints_client.req.b} = {response.sum}')

    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Actions: Long-Running Goal-Feedback-Result Communication

Actions provide a more complex and flexible communication pattern for long-running, interruptible tasks. Unlike services, actions allow for continuous feedback during execution and can be preempted. This is vital for robot behaviors like navigation, arm movements, or complex manipulation tasks.

Key characteristics of ROS 2 Actions:
- **Asynchronous**: Client doesn't block while waiting for the result.
- **Goal, Feedback, Result**: Client sends a goal, receives continuous feedback, and gets a final result.
- **Preemptable**: Clients can cancel a goal.
- **Action Definition**: Actions have a `.action` file defining the goal, result, and feedback message structure.

### Practical Example: Fibonacci Action

An action to compute a Fibonacci sequence up to a given order.

#### Action Definition (`Fibonacci.action`)

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

#### Action Server Node (`fibonacci_action_server.py`)

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server Ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled!')
                return Fibonacci.Result()
            
            sequence.append(sequence[i] + sequence[i-1])
            
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Feedback: %r' % feedback_msg.partial_sequence)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Action Client Node (`fibonacci_action_client.py`)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci Action Client Ready.')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Further Reading

- [ROS 2 Concepts - Services](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [ROS 2 Concepts - Actions](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
