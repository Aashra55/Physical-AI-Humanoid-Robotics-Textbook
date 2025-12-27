# ROS 2 پیکجز بنانا اور لانچ کرنا

جیسے جیسے آپ کے ROS 2 منصوبوں کی پیچیدگی بڑھتی ہے، انفرادی نوڈس اور ان کے نفاذ کا انتظام مشکل ہو سکتا ہے۔ یہ باب ROS 2 پیکجز اور لانچ فائلوں کو متعارف کراتا ہے، جو متعدد نوڈس اور کنفیگریشنز کو منظم، بنانے اور چلانے کا ایک منظم طریقہ فراہم کرتے ہیں۔

## ROS 2 پیکجز کو سمجھنا

ایک ROS 2 پیکیج ROS 2 کوڈ کے لیے تنظیم کی بنیادی اکائی ہے۔ یہ سورس کوڈ، بلڈ اسکرپٹس، کنفیگریشن فائلیں، اور دیگر وسائل کو شامل کرتا ہے۔ پیکجز روبوٹکس سافٹ ویئر کو شیئر کرنا، دوبارہ استعمال کرنا، اور انتظام کرنا آسان بناتے ہیں۔

Key components of a ROS 2 package:
- `package.xml`: Defines package metadata, dependencies, and maintainers.
- `CMakeLists.txt` (for C++ packages) or `setup.py` (for Python packages): Build instructions.
- `src/`: Source code directory.
- `launch/`: Directory for launch files.
- `config/`: Directory for configuration files.
- `resource/`: Directory for shared resources.

### Creating a New ROS 2 Package (Python Example)

```bash
# Navigate to your ROS 2 workspace src directory
cd ~/ros2_ws/src

# Create a new Python package named 'my_robot_pkg'
ros2 pkg create --build-type ament_python my_robot_pkg
```

This command creates a basic package structure with `package.xml` and `setup.py` files.

## Building ROS 2 Packages with `colcon`

`colcon` is the primary build tool for ROS 2. It can build multiple packages in a workspace, respecting their dependencies.

### Building Your Workspace

```bash
# Navigate to your ROS 2 workspace root
cd ~/ros2_ws

# Build all packages in the workspace
colcon build

# To build only a specific package
colcon build --packages-select my_robot_pkg
```

After building, remember to `source` your workspace to make the new executables and libraries available:

```bash
source install/setup.bash
```

## Orchestrating Nodes with ROS 2 Launch Files

Launch files are XML or Python scripts that allow you to define and run a complex system of ROS 2 nodes, including parameters, remappings, and other configurations, all with a single command. They are invaluable for setting up a robot's entire software stack.

### Example Launch File (`my_nodes.launch.py`)

This Python-based launch file starts both the publisher and subscriber nodes from our previous example.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='publisher',
            name='my_publisher',
            output='screen'
        ),
        Node(
            package='my_robot_pkg',
            executable='subscriber',
            name='my_subscriber',
            output='screen'
        )
    ])
```

### Running a Launch File

```bash
ros2 launch my_robot_pkg my_nodes.launch.py
```

## مزید پڑھنا

- [ROS 2 ٹیوٹوریلز - ایک ورک اسپیس بنانا](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Creating-A-Workspace/Creating-A-Workspace.html)
- [ROS 2 ٹیوٹوریلز - ایک Python پیکیج بنانا](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Creating-Your-First-ROS2-Package.html)
- [ROS 2 ٹیوٹوریلز - لانچ فائلوں کا استعمال](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launch-Files/Creating-A-Launch-File.html)
