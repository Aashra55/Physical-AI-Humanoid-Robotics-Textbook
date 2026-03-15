# ROS 2 خدمات اور اعمال

نوڈس اور موضوعات پر تعمیر کرتے ہوئے، ROS 2 مخصوص تعامل کے نمونوں کے لیے اعلیٰ سطح کے مواصلاتی میکانزم بھی فراہم کرتا ہے: خدمات اور اعمال۔ یہ روبوٹکس سسٹم میں مضبوط اور منظم مواصلات کے لیے اہم ہیں۔

## ROS 2 خدمات: درخواست-جواب مواصلات

ROS 2 میں خدمات نوڈس کے درمیان ہم وقت ساز، درخواست-جواب مواصلاتی پیٹرن کو فعال کرتی ہیں۔ یہ ان کارروائیوں کے لیے مثالی ہے جہاں ایک کلائنٹ نوڈ کو سرور نوڈ سے ایک مخصوص حساب کتاب یا ڈیٹا کی ضرورت ہوتی ہے اور وہ جواب کا انتظار کرنے کا متحمل ہو سکتا ہے۔

ROS 2 خدمات کی اہم خصوصیات:
- **ہم وقت ساز**: کلائنٹ سرور کے جواب کا انتظار کرتا ہے۔
- **کلائنٹ-سرور ماڈل**: ایک نوڈ سروس سرور کے طور پر کام کرتا ہے، ایک سروس فراہم کرتا ہے، اور دوسرا کلائنٹ کے طور پر کام کرتا ہے، اس کی درخواست کرتا ہے۔
- **سروس کی تعریف**: خدمات میں ایک `.srv` فائل ہوتی ہے جو درخواست اور جواب کے پیغام کی ساخت کی تعریف کرتی ہے۔

### عملی مثال: سادہ ایڈر سروس

ایک ایسی سروس پر غور کریں جو درخواست کے طور پر دو انٹیجرز لیتی ہے اور ان کے مجموعہ کو جواب کے طور پر واپس کرتی ہے۔

#### سروس کی تعریف (`AddTwoInts.srv`)

```
int64 a
int64 b
---
int64 sum
```

#### سروس سرور نوڈ (`add_two_ints_server.py`)

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

#### سروس کلائنٹ نوڈ (`add_two_ints_client.py`)

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

## ROS 2 اعمال: طویل المدتی مقصد-فیڈ بیک-نتیجہ مواصلات

اعمال طویل المدتی، مداخلت کے قابل کاموں کے لیے ایک زیادہ پیچیدہ اور لچکدار مواصلاتی پیٹرن فراہم کرتے ہیں۔ خدمات کے برعکس، اعمال عمل کے دوران مسلسل فیڈ بیک کی اجازت دیتے ہیں اور انہیں روکا جا سکتا ہے۔ یہ روبوٹ کے رویے کے لیے اہم ہے جیسے نیویگیشن، بازو کی نقل و حرکت، یا پیچیدہ ہیرا پھیری کے کام۔

ROS 2 اعمال کی اہم خصوصیات:
- **غیر ہم وقت ساز**: کلائنٹ نتیجے کا انتظار کرتے ہوئے بلاک نہیں ہوتا ہے۔
- **مقصد، فیڈ بیک، نتیجہ**: کلائنٹ ایک مقصد بھیجتا ہے، مسلسل فیڈ بیک وصول کرتا ہے، اور ایک حتمی نتیجہ حاصل کرتا ہے۔
- **روکا جا سکتا ہے**: کلائنٹ ایک مقصد کو منسوخ کر سکتے ہیں۔
- **عمل کی تعریف**: اعمال میں ایک `.action` فائل ہوتی ہے جو مقصد، نتیجہ، اور فیڈ بیک کے پیغام کی ساخت کی تعریف کرتی ہے۔

### عملی مثال: فیبونیکی عمل

ایک عمل جو ایک دیئے گئے ترتیب تک فیبونیکی ترتیب کا حساب لگاتا ہے۔

#### عمل کی تعریف (`Fibonacci.action`)

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

#### عمل سرور نوڈ (`fibonacci_action_server.py`)

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

#### عمل کلائنٹ نوڈ (`fibonacci_action_client.py`)

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

## مزید پڑھنا

- [ROS 2 تصورات - خدمات](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [ROS 2 تصورات - اعمال](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
