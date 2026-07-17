# Drone Path Control API using DroneKit and MAVLink

This project is a Flask-based drone control API that uses DroneKit and MAVLink to execute grid-based drone movement paths.

The API receives a path as a list of grid coordinates and converts that path into drone movement actions such as takeoff, forward movement, left turn, right turn, hover, and landing.

The project supports two modes:

1. **Internal Simulation Mode** — for testing the path logic safely without connecting to a drone.
2. **Real Drone / Mission Planner Mode** — for connecting to a Pixhawk/MAVLink vehicle or Mission Planner SITL.

---

## Project Overview

This project is designed to control a drone through a REST API. A user or external application can send a grid-based path to the API, and the API will execute that path step by step.

Each point in the path represents a position on a grid. The drone starts from the first point, moves toward the next point, turns when needed, moves forward, hovers for the provided time, and finally lands after completing the full path.

This system is useful for testing basic drone path-following logic using Python, Flask, DroneKit, MAVLink, Mission Planner, and a Pixhawk 2.4.8 flight controller.

---

## Technologies Used

- Python
- Flask
- DroneKit
- pymavlink
- MAVLink
- Mission Planner
- Pixhawk 2.4.8 flight controller
- ArduPilot / PX4 compatible drone or simulator

---

## Hardware Used

This project was tested with a **Pixhawk 2.4.8 flight controller**.

Pixhawk 2.4.8 works with ArduPilot/PX4 based firmware and communicates using MAVLink. The Flask API connects to the flight controller through DroneKit and pymavlink using a serial, USB, telemetry, TCP, or UDP connection.

Common Pixhawk connection examples:

```text
/dev/ttyACM0        Linux USB connection
COM3               Windows USB/telemetry connection
127.0.0.1:14550    SITL / Mission Planner simulation connection
```

The actual connection string depends on the system, driver, telemetry module, and Mission Planner/SITL configuration.

---

## File Explanation

### api_fixed.py

This file contains the cleaned and fixed version of the original path-following API.

It is mainly useful for testing the path-following logic safely. It validates the incoming path, calculates the movement direction, and prints the actions such as turning, moving forward, hovering, and landing.

This file does not need to connect to a real drone.

### api_drone_full.py

This is the complete version of the API.

It supports both internal simulation mode and real drone mode.

In internal simulation mode, the code only prints the drone actions and does not connect to a real drone.

In real drone mode, the code connects to a Pixhawk 2.4.8 / ArduPilot / PX4 compatible drone through DroneKit and MAVLink and sends real flight commands.

---

## How the Project Works

The API receives a JSON request containing a path and hover times.

Example:

```json
{
  "path": [[0, 0], [1, 0], [1, 1], [2, 1]],
  "hover_time_list": [2, 3, 2, 4]
}
```

### Path Meaning

```text
[0, 0] = starting point
[1, 0] = move down
[1, 1] = move right
[2, 1] = move down
```

### Hover Time Meaning

```text
[0, 0] hover for 2 seconds
[1, 0] hover for 3 seconds
[1, 1] hover for 2 seconds
[2, 1] hover for 4 seconds
```

The drone follows the path point by point. At each point, it waits according to the value in `hover_time_list`.

---

## Drone Direction Logic

The drone direction is handled using four constants:

```python
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3
```

The drone keeps track of its current facing direction.

If the drone is already facing the required direction, it moves forward.

If the drone is not facing the required direction, it first turns left or right and then moves forward.

Example:

```text
Current direction: DOWN
Next required direction: RIGHT
Action: Turn, then move forward
```

The movement logic converts grid changes into drone directions:

```text
Row increases    = DOWN
Row decreases    = UP
Column increases = RIGHT
Column decreases = LEFT
```

---

## API Endpoints

### Home Endpoint

```text
GET /
```

This endpoint checks whether the API server is running.

Example:

```text
http://localhost:5000/
```

### Status Endpoint

```text
GET /status
```

This endpoint checks the drone/API status.

In internal simulation mode, it returns that no real drone is connected.

In real drone mode, it can return:

```text
battery
armed status
flight mode
altitude
connection status
```

Example:

```text
http://localhost:5000/status
```

### Path Execution Endpoint

```text
POST /path
```

This is the main endpoint. It receives the path and hover time list, then executes the movement.

Example URL:

```text
http://localhost:5000/path
```

Example JSON body:

```json
{
  "path": [[0, 0], [1, 0], [1, 1], [2, 1]],
  "hover_time_list": [2, 3, 2, 4]
}
```

### Emergency Landing Endpoint

```text
POST /land
```

This endpoint sends a landing command to the drone.

Example:

```text
http://localhost:5000/land
```

---

## Installation

Install the required Python packages:

```bash
pip install -r requirements.txt
```

Or install manually:

```bash
pip install flask dronekit pymavlink
```

---

## requirements.txt

```text
flask
dronekit
pymavlink
```

---

## Internal Simulation Mode

Internal simulation mode is used to test the path logic without Mission Planner or a real drone. It only prints the actions in the terminal.

### Windows

```bash
set SIMULATION_MODE=true
python api_drone_full.py
```

### Mac/Linux

```bash
export SIMULATION_MODE=true
python api_drone_full.py
```

Then open:

```text
http://localhost:5000/
```

This is useful for checking whether the API and path logic are working before connecting to Mission Planner or a real drone.

---

## Mission Planner Simulation

For MAVLink simulation, this project can be tested with **Mission Planner** and **ArduPilot**.

Mission Planner is useful because it allows the drone behaviour to be tested in a simulated environment before using a real drone.

### Step 1: Open Mission Planner

Open Mission Planner on your computer.

### Step 2: Start SITL Simulation

In Mission Planner, start a SITL simulation for a copter/drone.

Mission Planner will provide a simulated MAVLink vehicle connection, usually through a TCP/UDP port.

Common SITL connection examples:

```text
127.0.0.1:14550
tcp:127.0.0.1:5760
udp:127.0.0.1:14550
```

The exact connection string depends on how Mission Planner/SITL is configured.

### Step 3: Run the API with SITL

Even though this is a simulation, set `SIMULATION_MODE=false` because the API should connect to the simulated MAVLink vehicle.

Windows example:

```bash
set SIMULATION_MODE=false
set DRONE_CONNECTION=127.0.0.1:14550
set DRONE_BAUD=57600
python api_drone_full.py
```

Mac/Linux example:

```bash
export SIMULATION_MODE=false
export DRONE_CONNECTION=127.0.0.1:14550
export DRONE_BAUD=57600
python api_drone_full.py
```

### Step 4: Check API Status

Open:

```text
http://localhost:5000/status
```

If the API connects successfully, it should show drone status such as mode, altitude, armed status, or battery level.

### Step 5: Send a Test Path

Send a POST request to:

```text
http://localhost:5000/path
```

Example body:

```json
{
  "path": [[0, 0], [1, 0]],
  "hover_time_list": [2, 2]
}
```

This small test path should:

```text
1. Connect to the simulated drone
2. Switch to GUIDED mode
3. Arm the drone
4. Take off to 1 meter
5. Hover for 2 seconds
6. Move forward one grid step
7. Hover for 2 seconds
8. Land
```

---

## Real Drone Mode

Real drone mode connects to an actual ArduPilot compatible drone using DroneKit and MAVLink. In this project, the hardware used was a Pixhawk 2.4.8 flight controller.

### Windows Example

```bash
set SIMULATION_MODE=false
set DRONE_CONNECTION=COM3
set DRONE_BAUD=57600
python api_drone_full.py
```

### Linux Example

```bash
export SIMULATION_MODE=false
export DRONE_CONNECTION=/dev/ttyACM0
export DRONE_BAUD=57600
python api_drone_full.py
```

### Real Drone Test Request

Start with a very small path:

```json
{
  "path": [[0, 0], [1, 0]],
  "hover_time_list": [2, 2]
}
```

This will:

```text
1. Connect to the drone
2. Switch to GUIDED mode
3. Arm the drone
4. Take off to 1 meter
5. Hover for 2 seconds
6. Move forward one grid step
7. Hover for 2 seconds
8. Land
```

---

## Configuration

The project uses environment variables for configuration.

| Variable | Description | Default |
|---|---|---|
| `SIMULATION_MODE` | Enables or disables internal simulation mode | `true` |
| `DRONE_CONNECTION` | Drone connection string | `/dev/ttyACM0` |
| `DRONE_BAUD` | Drone baud rate | `57600` |
| `TAKEOFF_ALTITUDE` | Takeoff altitude in meters | `1.0` |
| `FORWARD_SPEED_MPS` | Forward speed in meters per second | `0.25` |
| `GRID_STEP_DURATION` | Duration for one grid step | `3.0` |
| `YAW_RATE` | Yaw rotation speed in degrees per second | `20` |

---

## Movement Settings

The default movement settings are:

```text
Takeoff altitude: 1 meter
Forward speed: 0.25 meters/second
Grid step duration: 3 seconds
Yaw rate: 20 degrees/second
```

This means one grid step roughly covers:

```text
0.25 × 3 = 0.75 meters
```

If one grid step should be closer to 1 meter, adjust the values:

```bash
set FORWARD_SPEED_MPS=0.25
set GRID_STEP_DURATION=4
```

This gives:

```text
0.25 × 4 = 1 meter
```

<!-- update -->
Currently polishing GitHub projects and documentation.

---

## Safety Notes

Before running in real drone mode:

- Test the code in internal simulation mode first.
- Test with Mission Planner before using a real drone.
- Remove propellers during bench testing.
- Make sure the drone is connected properly.
- Use a safe and open area for real flight.
- Keep a manual controller or emergency landing option ready.
- Start with a very small path.
- Tune speed and duration carefully.
- Do not test near people, vehicles, buildings, or obstacles.

---

Recommended topics:

```text
python
flask
dronekit
mavlink
pymavlink
drone
ardupilot
px4
uav
pixhawk
pixhawk-248
mission-planner
path-planning
simulation
```

---

## Disclaimer

This project is for educational and research purposes. Real drone testing should only be done in a safe environment with proper supervision and safety precautions.
