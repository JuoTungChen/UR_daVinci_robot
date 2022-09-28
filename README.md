# MOPS: A Modular and Open Platform for Surgical Robotics Research

Surgical robot platform for research based on standard robot manipulators with adapters for surgical instruments.

Target platform: Ubuntu 22.04 with ROS2 Humble/Rolling.


## Getting started

1. Build and install [RMLTypeII](https://gitlab.com/sdurobotics/medical/rmltypeii) (or RMLTypeIV) and [pyreflexxes](https://gitlab.com/sdurobotics/medical/pyreflexxes).
2. Install `colcon` and `vcs` tools
   ```bash
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```
3. Set up workspace folder and pull source code
   ```bash
   export MOPS_WS=~/rosws/mops
   mkdir -p $MOPS_WS/src
   cd $MOPS_WS/src
   git clone git@gitlab.com:sdurobotics/medical/mops/mops_core.git
   vcs import --input mops_ros/mops-public.repos
   ```
4. Install dependencies of packages in the workspace
   ```bash
   # TODO
   #rosdep update
   #rosdep install --from-paths . --ignore-src -y -r
   sudo apt install ros-humble-ur  # or ros-humble-ur
   ```
5. Build workspace
   ```bash
   cd $MOPS_WS
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

You may want to `touch $MOPS_WS/src/touch/touch_control/COLCON_IGNORE` which depends on OpenHaptics (`libHD.so`, `hd.h`), and Touch device driver (`libPhantomIOLib42.so`).
