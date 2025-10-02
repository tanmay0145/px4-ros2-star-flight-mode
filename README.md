# PX4-ROS2 Star Flight Mode

**Author:** Nathaniel Handan  
**Contact:** handanfoun@gmail.com  
**License:** BSD-3-Clause

This is a ROS 2 component that can be used to add a custom flight mode to a PX4 autopilot which will cause it to fly in the shape of a 5-pointed star. This is a Python implementation that demonstrates how to create custom flight modes using direct ROS 2 message publishing instead of the C++ px4_ros2 interface library.

## Requirements

- Ubuntu (22.04)
- PX4 and ROS 2 installations as detailed by https://docs.px4.io/main/en/ros2/user_guide.html
  - This involves setting up the PX4 toolchain, ROS 2, and the Micro XRCE-DDS Agent
- Tested on PX4 v1.15.0 and v1.15.4, ROS 2 Humble
- If this is a first time setup, try at least one of the examples at the end of the installation guide above before moving on to confirm you have everything set up properly.
- QGroundControl
  - The most recent available releases (daily and V4.4.3 as of writing) do not support dynamically adding external flight modes that are visible in the flight modes drop down menu in QGC. However, building the Stable_V4.4 branch manually does support this. It doesn't matter for this example, however, since the flight mode will activate automatically.

## Setup

### Setup and build a new ROS workspace
```bash
# Make a new folder for the workspace (if you don't already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this package
git clone https://github.com/Tinny-Robot/px4-ros2-star-flight-mode.git px4_python_star_flight_mode

# Build the workspace and setup the shell to run ros commands later
cd ..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Running the Flight Mode

1. In a new terminal, start MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

2. In another new terminal, start a Gazebo PX4 simulation
```bash
cd ~/PX4-Autopilot  # Change this depending on where you installed PX4

# Ignore these 2 lines if you are already on the correct version of PX4
git checkout v1.15.4
git submodule update --init --recursive

make px4_sitl gz_x500
```

3. Start QGroundControl. It should connect to the drone in the simulator.

4. Return to the first terminal and run the following. If you closed the terminal, you'll need to run the 2 source commands from the setup step again beforehand.
```bash
ros2 run px4_python_star_flight_mode star_flight_mode
```

The drone should automatically arm, takeoff, draw a 5-pointed star pattern, and then return to launch and land. QGC may display a weird flight mode value; ignore this.

## Flight Parameters

The flight parameters can be modified in the `StarFlightMode` class:
- `STAR_RADIUS`: 10.0 meters
- `FLIGHT_ALTITUDE`: -5.0 meters (NED frame) 
- `POSITION_TOLERANCE`: 1.0 meters