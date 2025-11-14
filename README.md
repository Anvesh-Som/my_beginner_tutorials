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
```

# Part 2

Minimal C++ pub/sub package extended for **Part 2** with:
- A **service** to change the talker’s base string.
- **All five logging levels** (DEBUG/INFO/WARN/ERROR/FATAL) using `_STREAM` macros.
- A **launch file** with CLI args (`freq_hz`, `base_string`) to modify node behavior.
- Doxygen file/class/method comments and an Apache-2.0 license.

---

## Features

**Topics**
- `topic` — `std_msgs/msg/String` published by talker, consumed by listener.

**Service**
- `/set_base_string` — `beginner_tutorials/srv/SetBaseString`
  - Request: `string base`
  - Response: `bool success`, `string message`

**Parameters (talker)**
- `base_string` (`string`, default: `"UMD Robotics says hi!"`)
- `publish_frequency_hz` (`double`, default: `2.0`)

**Logging**
- Uses DEBUG/INFO/WARN/ERROR/FATAL across both nodes via `_STREAM` macros.
- Works with `rqt_console` and `rqt_logger_level`.

**Launch**
- `bringup.launch.py` starts both nodes and accepts:
  - `freq_hz` (double)
  - `base_string` (string)

---

## Dependencies / Assumptions

- ROS 2 **Humble** on Ubuntu 22.04
- `colcon`, `ament_cmake`
- C++17 toolchain
- (optional for screenshots) `rqt_console`, `rqt_logger_level`

---

## Build

```bash
# From your existing colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/<your-user>/my_beginner_tutorials.git
cd ..
colcon build --packages-select beginner_tutorials
source install/setup.bash

