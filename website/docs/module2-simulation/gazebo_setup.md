# Setting up a Robot in Gazebo

Gazebo is a powerful 3D robotics simulator widely used in the ROS 2 ecosystem. It allows you to simulate complex robot scenarios in environments that are as physically accurate as possible. This chapter will guide you through setting up your robot model within Gazebo, enabling realistic simulation of its kinematics, dynamics, and sensor interactions.

## Prerequisites

Before setting up your robot in Gazebo, ensure you have:
-   **ROS 2 Humble** installed (as covered in Module 1).
-   **Gazebo** installed and integrated with ROS 2 (typically through `ros-humble-gazebo-ros-pkgs`).
-   A **URDF model** of your robot (as created in Module 1).

## Integrating URDF with Gazebo

Gazebo can directly use URDF files for robot descriptions. However, for full simulation capabilities, including physics properties and sensor plugins, URDF files are often augmented or converted to the **Simulation Description Format (SDF)**. The `gazebo_ros` packages provide tools and plugins to bridge the gap between URDF and Gazebo.

### Key Steps:

1.  **Augmenting URDF for Gazebo**:
    -   Add `<gazebo>` tags within your URDF to specify Gazebo-specific properties like materials, friction, and collision parameters.
    -   Integrate Gazebo plugins for differential drives, cameras, LIDARs, IMUs, etc. These plugins typically require a ROS 2 interface to publish sensor data or receive commands.

2.  **Creating a Gazebo World**:
    -   A Gazebo world (`.world` file) defines the environment, including ground planes, lighting, static objects (e.g., walls, furniture), and dynamic objects.
    -   Your robot model is then spawned into this world.

### Example: Spawning a URDF in a Simple Gazebo World

First, a minimal Gazebo world file (`empty.world`):

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

Next, a ROS 2 launch file to start Gazebo with this world and then spawn your URDF model:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your robot_description package
    robot_description_pkg = get_package_share_directory('simple_robot_description') # Assuming this is your package from Module 1
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'simple_arm.urdf') # Your URDF

    # Launch Gazebo
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo.launch.py')),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Node to publish the robot_description topic from the URDF file
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Node to spawn your robot model in Gazebo
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

## Controlling the Robot in Gazebo

Once your robot is spawned, you can interact with it using various ROS 2 tools and packages:
-   **`ros2_control`**: A generic control framework that can interface with both real and simulated hardware.
-   **Teleoperation**: Sending commands from a joystick or keyboard (e.g., using `teleop_twist_joy` or a custom node publishing to `/cmd_vel`).

## Further Reading

- [Gazebo ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launch-Files/Creating-A-Launch-File.html)
- [URDF for Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
- [ROS 2 Control Documentation](https://control.ros.org/master/doc/index.html)
