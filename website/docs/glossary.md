# Glossary

This glossary defines key terms used throughout the Physical AI & Humanoid Robotics Textbook.

## ROS 2

**Robot Operating System (ROS) 2**: A set of software libraries and tools for building robot applications. It provides services designed for a heterogeneous computer cluster, such as hardware abstraction, low-level device control, implementation of commonly used functionalities, message-passing between processes, and package management.

## URDF

**Unified Robot Description Format (URDF)**: An XML file format in ROS used for describing all elements of a robot. It represents the robot's physical structure, joints, and sensors, enabling visualization in tools like RViz and facilitating kinematics and dynamics calculations.

## SDF

**Simulation Description Format (SDF)**: An XML format used for describing environments and objects for robot simulators, primarily Gazebo. Unlike URDF, SDF can describe more complex environments, including multiple robots, static objects, physics properties, and sensor models.

## VSLAM

**Visual Simultaneous Localization and Mapping (VSLAM)**: A technique that allows a robot to simultaneously construct or update a map of an unknown environment while at the same time keeping track of its own position within that map, using only visual input from cameras.

## VLA

**Vision-Language-Action (VLA)**: A class of artificial intelligence models that integrate capabilities from computer vision (perceiving the world), natural language understanding (interpreting commands), and robotic action (executing tasks). VLA models bridge the gap between human instruction and robot behavior.

## Isaac Sim

**NVIDIA Isaac Sim**: A scalable robotics simulation and synthetic data generation platform built on NVIDIA Omniverse. It offers high-fidelity, photorealistic simulation with accurate physics, and tools for generating diverse, labeled datasets to train AI models for robots.

## Gazebo

**Gazebo**: A popular open-source 3D robotics simulator that allows for the accurate simulation of robots in complex indoor and outdoor environments. It provides a robust physics engine, a wide variety of sensors, and a convenient interface for integrating with ROS 2.

## Unity

**Unity**: A real-time 3D development platform often used for creating interactive simulations, games, and visualizations. In robotics, Unity can be used for high-fidelity rendering, human-robot interaction (HRI), and as a simulation environment, often integrating with ROS 2 via specialized packages.
