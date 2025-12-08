# Quickstart: Setting Up Your Development Environment

This guide provides the steps to set up a development environment for the Physical AI & Humanoid Robotics textbook projects. The primary target platform is **Ubuntu 22.04**, which is required for full compatibility with ROS 2 Humble and the NVIDIA Isaac tools.

## Prerequisites

- A computer meeting the minimum hardware requirements outlined in the `spec.md`, particularly an NVIDIA RTX GPU.
- Ubuntu 22.04 installed.
- Familiarity with the Linux command line.

## Step 1: Install Core Dependencies

First, install essential tools like Git, Docker, and common build utilities.

```bash
sudo apt update
sudo apt install -y git build-essential docker.io
```

## Step 2: Install NVIDIA Drivers and CUDA

A compatible NVIDIA driver and CUDA toolkit are essential for leveraging the GPU in simulation and for AI tasks.

1.  **Install NVIDIA Drivers**: It is recommended to install the latest proprietary drivers via the "Additional Drivers" tool in Ubuntu or from the command line.
2.  **Install CUDA Toolkit**: Follow the official NVIDIA guide to install the CUDA Toolkit. For ROS 2 Humble, CUDA 11.7 or 11.8 is a safe choice.

## Step 3: Install ROS 2 Humble

Follow the official ROS 2 documentation to install ROS 2 Humble Hawksbill.

```bash
# Follow instructions at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# ... (commands for setting locale, sources, etc.)

# Install ROS 2 Humble Desktop
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools

# Source the setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: Install Gazebo

Gazebo is included with the `ros-humble-desktop` installation. To install it separately or to ensure you have the latest version compatible with ROS 2:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Step 5: Set up the Project Repository

Clone the project repository and initialize the workspace.

```bash
# Clone the repository
git clone <repository-url>
cd <repository-directory>

# Build the Docusaurus website (optional, for viewing the textbook)
cd website
npm install
cd ..

# Build the ROS 2 projects
cd projects
colcon build
```

## Step 6: Install NVIDIA Isaac Sim (Recommended)

For Modules 3 and 4, NVIDIA Isaac Sim is required.

1.  Download and install **NVIDIA Omniverse Launcher**.
2.  From the Omniverse Launcher, install **Isaac Sim**. The recommended version will be specified in Module 3.
3.  Follow the Isaac Sim documentation to enable the ROS 2 Humble bridge.

After completing these steps, your environment will be ready for the projects in this book. Each module's `README.md` file will contain specific instructions for running its associated projects.
