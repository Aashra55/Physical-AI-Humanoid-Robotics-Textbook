# Working with SDF and URDF in Simulation

In ROS 2 and Gazebo, robot models are typically defined using either URDF (Unified Robot Description Format) or SDF (Simulation Description Format). While both describe robots, they serve slightly different purposes and have distinct capabilities, especially when it comes to simulation environments like Gazebo. Understanding when and how to use each is crucial for effective robotics development.

## URDF: Unified Robot Description Format

As introduced in Module 1, URDF is an XML format for describing the kinematic and dynamic properties of a robot. It's primarily designed for ROS to represent the robot's structure, joints, and sensors for things like:
-   **Kinematics**: Forward and inverse kinematics.
-   **Visualization**: Displaying the robot in `RViz`.
-   **State Estimation**: Processing sensor data relative to the robot's structure.
-   **Collision Checking**: Basic collision geometry.

**Strengths of URDF**: Simplicity, widely adopted in ROS, good for abstract robot descriptions.
**Limitations of URDF**: Cannot describe environments, multi-robot systems, or certain advanced physics properties directly.

## SDF: Simulation Description Format

SDF is a more comprehensive XML format designed specifically for describing objects and environments in simulation. It's the native format for Gazebo and can describe a much wider range of features than URDF, including:
-   **Full Environments**: Static and dynamic objects, terrain, lighting, atmospheric conditions.
-   **Multiple Robots**: Easily define and manage multiple robots within a single world file.
-   **Advanced Physics Properties**: Detailed friction models, damping, spring-damper joints.
-   **Sensor Properties**: More intricate sensor models (e.g., noise, distortion).
-   **Plugins**: Directly embed Gazebo plugins for custom behaviors.

**Strengths of SDF**: Powerful for full simulation environments, native to Gazebo, describes complex physics.
**Limitations of SDF**: Can be more verbose and complex than URDF, less direct support in core ROS tools for kinematic parsing.

## URDF vs. SDF: When to Use Which?

-   **Use URDF when**: You need a concise description of a single robot for ROS-specific tools (e.g., `robot_state_publisher`, `RViz`).
-   **Use SDF when**: You need to describe a complete simulation environment, including multiple robots, static objects, and complex physics interactions in Gazebo.

**The Hybrid Approach**: Often, the best practice is to describe your robot primarily in URDF for ROS compatibility and then either:
1.  **Augment URDF with `<gazebo>` tags**: Add Gazebo-specific properties directly within the URDF.
2.  **Convert URDF to SDF**: Use tools (or implicitly via Gazebo's spawning mechanisms) to convert the URDF to an SDF model when loading it into Gazebo. This allows Gazebo to apply its full feature set while keeping the original robot definition concise in URDF.

## Example: Augmenting URDF for Gazebo

You can embed `<gazebo>` tags within your URDF to add simulation-specific properties.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <!-- ... URDF visual, collision, inertial ... -->
  </link>

  <joint name="base_joint" type="fixed">
    <!-- ... URDF parent, child, origin ... -->
  </joint>

  <!-- Gazebo specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
    <self_collide>true</self_collide>
  </gazebo>

  <!-- Gazebo plugin for a camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30.0</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
          <topic_name>image_raw</topic_name>
          <camera_info_topic_name>camera_info</camera_info_topic_name>
        </ros>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Further Reading

- [Gazebo SDF Specification](http://sdformat.org/spec)
- [ROS 2 Tutorials - URDF for Gazebo](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Gazebo-URDF-Integration.html)
- [Gazebo Plugins](http://gazebosim.org/tutorials?tut=plugins_overview&cat=write_plugin)
