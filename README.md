# beginner_tutorials (ENPM700 Assigment)

Minimal C++ publisher/subscriber following Google C++ style for Part 1.

## Assumptions / Dependencies
- ROS 2 Humble
- colcon, ament_cmake
- C++17 compiler (GCC or Clang)

## Build
```bash
# From the *workspace root* (NOT this repo root), e.g. ~/ros2_ws
mkdir -p src && cd src
# Clone this repo (contains only the package)
git clone https://github.com/<your-user>/my_beginner_tutorials.git
cd ..
colcon build --packages-select beginner_tutorials
source install/setup.bash

## New in Part 2
- **Service**: `set_base_string` (`beginner_tutorials/srv/SetBaseString`) to change talker base text.
- **Logging**: Uses all five levels via `_STREAM` macros across both nodes (DEBUG, INFO, WARN, ERROR, FATAL).
- **Launch**: `bringup.launch.py` runs both nodes; args `freq_hz` and `base_string`.

## Build
```bash
# In an existing colcon workspace (e.g., ~/ros2_ws)
cd ~/ros2_ws/src
git clone https://github.com/<your-user>/my_beginner_tutorials.git
# Ensure only the package directory exists in repo (as required)
colcon build --packages-select beginner_tutorials
source ~/ros2_ws/install/setup.bash

