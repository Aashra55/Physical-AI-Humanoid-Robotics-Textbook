# گزیبو میں ایک روبوٹ قائم کرنا

گزیبو ایک طاقتور 3D روبوٹکس سمیلیٹر ہے جو ROS 2 ایکو سسٹم میں وسیع پیمانے پر استعمال ہوتا ہے۔ یہ آپ کو ممکنہ حد تک جسمانی طور پر درست ماحول میں پیچیدہ روبوٹ منظرناموں کی سیمولیشن کرنے کی اجازت دیتا ہے۔ یہ باب آپ کو گزیبو کے اندر اپنے روبوٹ ماڈل کو ترتیب دینے میں رہنمائی کرے گا، جو اس کی کائینی میٹکس، ڈائنامکس، اور سینسر تعاملات کی حقیقت پسندانہ سیمولیشن کو فعال کرے گا۔

## پیشگی ضروریات

گزیبو میں اپنے روبوٹ کو ترتیب دینے سے پہلے، یقینی بنائیں کہ آپ کے پاس یہ ہیں:
-   **ROS 2 Humble** انسٹال ہے (جیسا کہ ماڈیول 1 میں شامل ہے)۔
-   **گزیبو** انسٹال ہے اور ROS 2 کے ساتھ مربوط ہے (عام طور پر `ros-humble-gazebo-ros-pkgs` کے ذریعے)۔
-   آپ کے روبوٹ کا **URDF ماڈل** (جیسا کہ ماڈیول 1 میں بنایا گیا ہے)۔

## گزیبو کے ساتھ URDF کو مربوط کرنا

گزیبو روبوٹ کی تفصیلات کے لیے براہ راست URDF فائلوں کا استعمال کر سکتا ہے۔ تاہم، مکمل سیمولیشن صلاحیتوں کے لیے، بشمول طبیعیات کی خصوصیات اور سینسر پلگ ان، URDF فائلوں کو اکثر بڑھایا جاتا ہے یا **سیمولیشن ڈسکرپشن فارمیٹ (SDF)** میں تبدیل کیا جاتا ہے۔ `gazebo_ros` پیکجز URDF اور گزیبو کے درمیان فرق کو پر کرنے کے لیے ٹولز اور پلگ ان فراہم کرتے ہیں۔

### اہم اقدامات:

1.  **گزیبو کے لیے URDF کو بڑھانا**:
    -   اپنے URDF کے اندر `<gazebo>` ٹیگز شامل کریں تاکہ گزیبو کی مخصوص خصوصیات جیسے مواد، رگڑ، اور تصادم کے پیرامیٹرز کی وضاحت کی جا سکے۔
    -   تفریقی ڈرائیوز، کیمرے، LIDARs، IMUs وغیرہ کے لیے گزیبو پلگ انز کو مربوط کریں۔ ان پلگ انز کو عام طور پر سینسر ڈیٹا کو شائع کرنے یا کمانڈز وصول کرنے کے لیے ROS 2 انٹرفیس کی ضرورت ہوتی ہے۔

2.  **ایک گزیبو ورلڈ بنانا**:
    -   ایک گزیبو ورلڈ (`.world` فائل) ماحول کی تعریف کرتی ہے، بشمول زمینی جہاز، روشنی، جامد اشیاء (مثال کے طور پر، دیواریں، فرنیچر)، اور متحرک اشیاء۔
    -   آپ کا روبوٹ ماڈل پھر اس دنیا میں سپون کیا جاتا ہے۔

### مثال: ایک سادہ گزیبو ورلڈ میں ایک URDF کو سپون کرنا

پہلے، ایک کم سے کم گزیبو ورلڈ فائل (`empty.world`):

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

اگلا، اس ورلڈ کے ساتھ گزیبو شروع کرنے اور پھر اپنے URDF ماڈل کو سپون کرنے کے لیے ایک ROS 2 لانچ فائل:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # آپ کے robot_description پیکیج کا راستہ
    robot_description_pkg = get_package_share_directory('simple_robot_description') # فرض کریں کہ یہ ماڈیول 1 سے آپ کا پیکیج ہے
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'simple_arm.urdf') # آپ کا URDF

    # گزیبو لانچ کریں
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo.launch.py')),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # URDF فائل سے robot_description موضوع کو شائع کرنے کے لیے نوڈ
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # گزیبو میں اپنے روبوٹ ماڈل کو سپون کرنے کے لیے نوڈ
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_arm', '-x', '0', '-y', '0', '-z', '1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```

## گزیبو میں روبوٹ کو کنٹرول کرنا

ایک بار جب آپ کا روبوٹ سپون ہو جاتا ہے، تو آپ مختلف ROS 2 ٹولز اور پیکجز کا استعمال کرتے ہوئے اس کے ساتھ تعامل کر سکتے ہیں:
-   **`ros2_control`**: ایک عام کنٹرول فریم ورک جو حقیقی اور سیمولیٹڈ ہارڈویئر دونوں کے ساتھ انٹرفیس کر سکتا ہے۔
-   **ٹیلی آپریشن**: جوائس اسٹک یا کی بورڈ سے کمانڈز بھیجنا (مثال کے طور پر، `teleop_twist_joy` یا `/cmd_vel` پر شائع کرنے والا ایک کسٹم نوڈ کا استعمال کرتے ہوئے)۔

## مزید پڑھنا

- [گزیبو ROS 2 ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launch-Files/Creating-A-Launch-File.html)
- [گزیبو کے لیے URDF](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
- [ROS 2 کنٹرول دستاویزات](https://control.ros.org/master/doc/index.html)
