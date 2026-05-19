# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS Noetic catkin workspace for UAV (drone) control systems. The main package `coffeeproj_pkg` is a coffee delivery UAV control package that handles MQTT communication, flight control via MAVROS, and task execution through a state machine.

## Build Commands

```bash
# Build specific package
catkin_make --pkg coffeeproj_pkg

# Build all packages
catkin_make

# Source the workspace after build
source devel/setup.bash
```

## Running

```bash
# Start the main coffee delivery node
./start_coffee.sh

# Or launch directly with parameters
roslaunch coffeeproj_pkg coffee_proj.launch broker_ip:=<ip> drone_id:=<id>

# Start testtool
./start_testtool.sh
```

## Prerequisites

Before running the UAV system:
1. `roscore` - ROS master
2. PX4 SITL (simulation): `make px4_sitl_default gazebo-classic`
3. MAVROS: `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" use_sim_time:=false`

## Architecture

### Main Package: coffeeproj_pkg

The main node (`coffee_proj_node.py`) orchestrates the following components:

- **CoffeeProjNode**: Main ROS node that initializes and coordinates all components
- **MqttClient** (`mqtt_client.py`): MQTT communication with broker
  - Subscribes: `drone/{id}/task`, `drone/{id}/command`
  - Publishes: `drone/{id}/telemetry`, `drone/{id}/status`
- **FlightController** (`flight_controller.py`): MAVROS interface for flight control
  - Arming/disarming, mode switching
  - Position control (local and global)
  - Takeoff, landing, waypoint navigation
- **TaskStateMachine** (`state_machine.py`): Manages task execution lifecycle
  - States: IDLE → LOADED → TAKEOFF → FLYING → ARRIVED → LANDING → LANDED → UNLOADED → RTL_FLYING → RTL_LANDING → COMPLETED
- **ZerooneRtk** (`zeroone_rtk.py`): RTK data forwarding to serial port

### Key Launch Parameters

The launch file (`coffee_proj.launch`) accepts:
- `broker_ip`, `broker_port`: MQTT broker connection
- `drone_id`, `mqtt_user`, `mqtt_pw`: Drone identification
- `takeoff_alt`, `cruise_speed`, `waypoint_tolerance`: Flight parameters
- `rtk_enabled`, `rtk_serial_port`: RTK configuration

### Other Packages

- `testtool_pkg`: MAVROS testing utilities
- `mavros_test_pkg`: C++ MAVROS test nodes
- `server_communicate_pkg`, `topic_communicate_pkg`, `testenv_pkg`: Supporting packages

## Code Conventions

- Python scripts are in `scripts/` directories with `#!/usr/bin/env python3` shebang
- Launch files define all configurable parameters with sensible defaults
- The main node passes itself to child components for parameter access and callbacks
- Coordinate system: ENU (East-North-Up) local frame, GPS for global positioning
