# ROS 2 نوڈس اور موضوعات

یہ باب ROS 2 میں دو بنیادی تصورات: نوڈس اور موضوعات کا گہرائی سے مطالعہ کرتا ہے۔ ان تصورات کو سمجھنا کسی بھی ROS 2 ایپلی کیشن کی تعمیر کے لیے بہت ضروری ہے، کیونکہ وہ مواصلاتی گراف کی بنیاد بناتے ہیں۔

## ROS 2 نوڈس کیا ہیں؟

ROS 2 میں، ایک نوڈ ایک قابل عمل پروگرام ہے جو ایک مخصوص کام انجام دیتا ہے۔ نوڈس کو عام طور پر ماڈیولر اور دوبارہ قابل استعمال بنانے کے لیے ڈیزائن کیا جاتا ہے۔ مثال کے طور پر، ایک روبوٹ میں موٹرز کو کنٹرول کرنے کے لیے ایک نوڈ، سینسر ڈیٹا پڑھنے کے لیے دوسرا، اور کیمرہ امیجز کو پروسیس کرنے کے لیے تیسرا نوڈ ہو سکتا ہے۔

ROS 2 نوڈس کی اہم خصوصیات:
- **ماڈیولرٹی**: ہر نوڈ ایک واحد، اچھی طرح سے بیان کردہ فنکشن انجام دیتا ہے۔
- **آزادی**: نوڈس علیحدہ پروسیسز کے طور پر چلتے ہیں اور مختلف پروگرامنگ زبانوں (مثال کے طور پر، Python، C++) میں لکھے جا سکتے ہیں۔
- **مواصلات**: نوڈس مختلف میکانزم کے ذریعے ایک دوسرے کے ساتھ بات چیت کرتے ہیں، بنیادی طور پر موضوعات کے ذریعے۔

## ROS 2 موضوعات کو سمجھنا

موضوعات ROS 2 میں غیر ہم وقت ساز، کئی سے کئی مواصلات کے لیے ایک طاقتور میکانزم ہیں۔ ایک نوڈ ایک موضوع پر پیغامات "شائع" کر سکتا ہے، اور دوسرے نوڈس ان پیغامات کو وصول کرنے کے لیے اس موضوع کو "سبسکرائب" کر سکتے ہیں۔

ROS 2 موضوعات کی اہم خصوصیات:
- **پبلش/سبسکرائب ماڈل**: غیر منسلک مواصلات، پبلشرز کو سبسکرائبرز کے بارے میں جاننے کی ضرورت نہیں ہے، اور اس کے برعکس۔
- **پیغام کی اقسام**: ہر موضوع کی ایک مخصوص پیغام کی قسم ہوتی ہے، جو ڈیٹا کی مستقل مزاجی کو یقینی بناتی ہے۔
- **ڈیٹا سٹریمز**: موضوعات ڈیٹا کے مسلسل سٹریمز کے لیے مثالی ہیں، جیسے سینسر ریڈنگز یا موٹر کمانڈز۔

## عملی مثال: پبلشر اور سبسکرائبر نوڈس

آئیے نوڈس اور موضوعات کو ایک سادہ Python مثال کے ساتھ واضح کریں: ایک "ٹاکر" نوڈ جو ایک موضوع پر پیغامات شائع کرتا ہے، اور ایک "سننے والا" نوڈ جو اس موضوع کو سبسکرائب کرتا ہے اور ان پیغامات کو وصول کرتا ہے۔

### ٹاکر نوڈ (`publisher.py`)

یہ نوڈ ہر سیکنڈ میں ایک موضوع پر ایک سادہ "ہیلو ROS 2" پیغام شائع کرے گا۔

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

### لسنر نوڈ (`subscriber.py`)

یہ نوڈ 'موضوع' کو سبسکرائب کرے گا اور وصول ہونے والے پیغامات کو پرنٹ کرے گا۔

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

## مثال کو چلانا

ان نوڈس کو چلانے کے لیے، آپ عام طور پر انہیں ROS 2 ورک اسپیس کے اندر مرتب کریں گے اور پھر انہیں `ros2 run` کمانڈز یا لانچ فائل کا استعمال کرتے ہوئے لانچ کریں گے۔

```bash
# مثال کمانڈز (ان نوڈس پر مشتمل ROS 2 پیکیج بنانے کے بعد)
ros2 run my_package minimal_publisher_node
ros2 run my_package minimal_subscriber_node
```

## مزید پڑھنا

- [ROS 2 تصورات - نوڈس](https://docs.ros.org/en/humble/Concepts/About-Nodes.html)
- [ROS 2 تصورات - موضوعات](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
