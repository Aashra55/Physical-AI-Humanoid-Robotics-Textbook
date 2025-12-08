# Building and Launching ROS 2 Packages

As your ROS 2 projects grow in complexity, managing individual nodes and their execution can become cumbersome. This chapter introduces ROS 2 packages and launch files, which provide a structured way to organize, build, and run multiple nodes and configurations.

## Understanding ROS 2 Packages

A ROS 2 package is the fundamental unit of organization for ROS 2 code. It encapsulates source code, build scripts, configuration files, and other resources. Packages make it easier to share, reuse, and manage robotics software.

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

## Further Reading

- [ROS 2 Tutorials - Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Creating-A-Workspace/Creating-A-Workspace.html)
- [ROS 2 Tutorials - Creating a Python Package](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Tutorials - Using Launch Files](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launch-Files/Creating-A-Launch-File.html)
