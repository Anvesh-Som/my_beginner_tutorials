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
