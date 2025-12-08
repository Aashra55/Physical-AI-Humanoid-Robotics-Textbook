# ROS 2 Nodes and Topics

This chapter delves into two fundamental concepts in ROS 2: Nodes and Topics. Understanding these concepts is crucial for building any ROS 2 application, as they form the basis of the communication graph.

## What are ROS 2 Nodes?

In ROS 2, a node is an executable program that performs a specific task. Nodes are typically designed to be modular and reusable. For instance, a robot might have one node for controlling motors, another for reading sensor data, and a third for processing camera images.

Key characteristics of ROS 2 Nodes:
- **Modularity**: Each node performs a single, well-defined function.
- **Independence**: Nodes run as separate processes and can be written in different programming languages (e.g., Python, C++).
- **Communication**: Nodes communicate with each other through various mechanisms, primarily topics.

## Understanding ROS 2 Topics

Topics are a powerful mechanism in ROS 2 for asynchronous, many-to-many communication. A node can "publish" messages to a topic, and other nodes can "subscribe" to that topic to receive the messages.

Key characteristics of ROS 2 Topics:
- **Publish/Subscribe Model**: Decoupled communication, publishers don't need to know about subscribers, and vice versa.
- **Message Types**: Each topic has a specific message type, ensuring data consistency.
- **Data Streams**: Topics are ideal for continuous streams of data, like sensor readings or motor commands.

## Practical Example: Publisher and Subscriber Nodes

Let's illustrate nodes and topics with a simple Python example: a "talker" node that publishes messages to a topic, and a "listener" node that subscribes to and receives those messages.

### Talker Node (`publisher.py`)

This node will publish a simple "Hello ROS 2" message to a topic every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Listener Node (`subscriber.py`)

This node will subscribe to the 'topic' and print the messages it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Example

To run these nodes, you would typically compile them within a ROS 2 workspace and then launch them using `ros2 run` commands or a launch file.

```bash
# Example commands (after building a ROS 2 package containing these nodes)
ros2 run my_package minimal_publisher_node
ros2 run my_package minimal_subscriber_node
```

## Further Reading

- [ROS 2 Concepts - Nodes](https://docs.ros.org/en/humble/Concepts/About-Nodes.html)
- [ROS 2 Concepts - Topics](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
