# MOPS Installation Guide

This guide provides instructions for installing MOPS (Medical Open Platform System) on Ubuntu 22.04 with ROS 2 Humble.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble

## Installation Steps

### 1. Enable Real-Time Kernel for ur_rtde

Follow the [Real-Time Setup Guide](https://sdurobotics.gitlab.io/ur_rtde/guides/guides.html#realtime-setup-guide) to enable the real-time kernel.

### 2. Setup Ubuntu Pro Account

1. Setup an Ubuntu Pro Account
2. Retrieve your free token
3. Attach the token to your device:
   ```
   sudo pro attach [YOUR_TOKEN]
   ```
4. Activate the real-time kernel:
   ```
   sudo ua enable realtime-kernel --beta
   ```

### 3. Setup Workspace

1. Create a workspace and src folder

### 4. Build and Install Dependencies

1. Clone and build RMLTypeII:
   ```
   cd src/
   git clone https://gitlab.com/sdurobotics/medical/rmltypeii.git
   ```

2. Clone and build pyreflexxes:
   ```
   git clone https://gitlab.com/sdurobotics/medical/pyreflexxes.git
   cd pyreflexxes/
   python3 setup.py build
   ```

3. Build the workspace:
   ```
   cd /path/to/workspace/root
   colcon build --symlink-install
   ```

### 5. Install Necessary Tools and Libraries

```
sudo apt install python3-colcon-common-extensions python3-vcstool librange-v3-dev
```

### 6. Clone and Setup MOPS Core

1. Clone MOPS core:
   ```
   cd src/
   git clone https://gitlab.com/sdurobotics/medical/mops/mops_core.git
   ```

2. Modify `mops-public.repos`: Change all `git@gitlab.com:...` to `https://gitlab.com/`

3. Import MOPS repositories:
   ```
   vcs import --input mops_core/mops-public.repos
   ```

4. Remove mops_ros if it appears in src:
   ```
   sudo rm -r mops_ros
   ```

5. Install ROS 2 UR package:
   ```
   sudo apt install ros-humble-ur
   ```

6. Add COLCON_IGNORE to touch_control:
   ```
   touch ./src/touch/touch_control/COLCON_IGNORE
   ```

### 7. Build MOPS Packages

1. Source ROS 2:
   ```
   source /opt/ros/humble/setup.bash
   ```

2. Build the packages:
   ```
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

## Robotis Dynamixel SDK Installation Guide

1. Setup workspace and src folder

2. Clone Dynamixel SDK:
   ```
   cd src/
   git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   ```

3. Build the workspace:
   ```
   cd /path/to/workspace
   colcon build --symlink-install
   ```

4. Connect U2D2 to the computer and check if `/dev/ttyUSB0` is present:
   ```
   ls /dev/tty*
   ```

5. Add permission to ports for the user account:
   ```
   sudo usermod -aG dialout [USER NAME]
   ```

6. Reboot to update permissions

7. Source the local setup:
   ```
   . install/local_setup.bash
   ```

### Testing Dynamixel SDK

1. Run the read_write_node:
   ```
   ros2 run dynamixel_sdk_examples read_write_node
   ```

2. In another terminal, source the path and run:
   ```
   ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 1, position: 1000}"
   ```

## Troubleshooting

If you encounter any issues during the installation process, please refer to the official documentation or contact the MOPS support team.