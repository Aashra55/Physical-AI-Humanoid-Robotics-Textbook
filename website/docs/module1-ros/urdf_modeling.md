# Modeling a Humanoid with URDF

To accurately simulate and control a robot, we first need a precise digital representation of its physical structure. The Unified Robot Description Format (URDF) is a powerful XML format used in ROS 2 to describe a robot's kinematics, dynamics, and visual appearance.

## What is URDF?

URDF allows you to define a robot as a set of rigid bodies (links) connected by joints. It can describe:
-   **Links**: The physical parts of the robot (e.g., torso, arm, hand). Each link has a visual representation (e.g., mesh, primitive shape), collision properties (for physics simulation), and inertial properties (mass, inertia tensor).
-   **Joints**: Connect links and define their relative motion (e.g., revolute, prismatic, fixed).
-   **Sensors**: Can be attached to links.

## Structure of a URDF File

A URDF file is an XML document starting with a `<robot>` tag. Inside, you define links and joints.

### Example: A Simple Robotic Arm Segment

```xml
<?xml version="1.0"?>
<robot name="simple_arm_segment">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.000416" ixy="0" ixz="0" iyy="0.000416" iyz="0" izz="0.000416"/>
    </inertial>
  </link>

  <!-- Joint connecting base_link to link1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000033" ixy="0" ixz="0" iyy="0.000033" iyz="0" izz="0.000002"/>
    </inertial>
  </link>

  <material name="blue"/>
  <material name="red"/>

</robot>
```

## Modeling a Humanoid Robot

Modeling a humanoid robot with URDF involves defining numerous links (torso, head, upper arms, forearms, hands, upper legs, lower legs, feet) and connecting them with appropriate joints (e.g., revolute joints for most human-like movements). This can become quite complex, often requiring modular URDF files that are then combined.

Key considerations for humanoid URDF:
-   **Degrees of Freedom (DoF)**: Humanoids have many DoF, making modeling and control challenging.
-   **Collision Geometry**: Accurate collision models are crucial for robust simulation and path planning.
-   **Inertial Properties**: Correct mass and inertia tensors are essential for realistic physics simulation.
-   **Meshes**: Often, external 3D model files (e.g., `.dae`, `.stl`) are used for visual and collision geometries.

## Visualizing URDF with `RViz`

`RViz` (ROS Visualization) is a powerful 3D visualizer for ROS 2. It can display URDF models, sensor data, and robot states, which is indispensable for debugging and understanding your robot's configuration.

```bash
# Example command to launch RViz and display a URDF model
ros2 launch urdf_tutorial display.launch.py model:=path/to/my_robot.urdf.xacro
```

## Best Practices for URDF

-   **Modular URDF**: Break down complex robots into smaller, reusable URDF or Xacro files.
-   **Xacro**: Use Xacro (`.urdf.xacro`) to simplify URDF files with macros, properties, and conditionals.
-   **Coordinate Frames**: Maintain consistent coordinate frames (e.g., Z-up, X-forward).
-   **Validation**: Use tools like `check_urdf` to validate your URDF files.

## Further Reading

- [ROS 2 Tutorials - URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [URDF XML Schema](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
