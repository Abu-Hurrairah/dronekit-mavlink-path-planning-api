from flask import Flask, request, jsonify
import os
import time
from typing import List, Tuple

try:
    from dronekit import connect, VehicleMode
    from pymavlink import mavutil
except ImportError:
    connect = None
    VehicleMode = None
    mavutil = None


app = Flask(__name__)

# ============================================================
# CONFIG
# ============================================================
# Use SIMULATION_MODE=True if you only want to test logic.
# Use SIMULATION_MODE=False when you are connected to a real Pixhawk/MAVLink drone
# or Mission Planner / ArduPilot SITL.
SIMULATION_MODE = os.getenv("SIMULATION_MODE", "true").lower() == "true"

# Examples:
# Linux USB telemetry: /dev/ttyACM0
# Windows COM port: COM3
# Mission Planner / SITL: 127.0.0.1:14550
CONNECTION_STRING = os.getenv("DRONE_CONNECTION", "/dev/ttyACM0")
BAUD_RATE = int(os.getenv("DRONE_BAUD", "57600"))

# Flight behaviour
TAKEOFF_ALTITUDE_METERS = float(os.getenv("TAKEOFF_ALTITUDE", "1.0"))
FORWARD_SPEED_MPS = float(os.getenv("FORWARD_SPEED_MPS", "0.25"))
GRID_STEP_DURATION_SECONDS = float(os.getenv("GRID_STEP_DURATION", "3.0"))
YAW_RATE_DEG_PER_SEC = float(os.getenv("YAW_RATE", "20"))

# Direction constants
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

DIRECTION_NAMES = {
    UP: "UP",
    DOWN: "DOWN",
    LEFT: "LEFT",
    RIGHT: "RIGHT",
}

vehicle = None


# ============================================================
# DRONE CONNECTION
# ============================================================

def connect_vehicle():
    """
    Connect to the drone once and reuse the connection.
    In simulation mode, no real drone connection is made.
    """
    global vehicle

    if SIMULATION_MODE:
        print("[SIMULATION] Drone connection skipped.")
        return None

    if connect is None or VehicleMode is None or mavutil is None:
        raise RuntimeError(
            "DroneKit/pymavlink is not installed. Run: pip install dronekit pymavlink"
        )

    if vehicle is not None:
        return vehicle

    print(f"[DRONE] Connecting to vehicle on {CONNECTION_STRING} at baud {BAUD_RATE}...")
    vehicle = connect(CONNECTION_STRING, baud=BAUD_RATE, wait_ready=True, timeout=60)
    print("[DRONE] Connected successfully.")
    return vehicle


def close_vehicle():
    global vehicle
    if vehicle is not None:
        print("[DRONE] Closing vehicle connection.")
        vehicle.close()
        vehicle = None


# ============================================================
# DRONE BASIC ACTIONS
# ============================================================

def wait_until_mode(mode_name: str, timeout: int = 30):
    if SIMULATION_MODE:
        print(f"[SIMULATION] Waiting for mode {mode_name} skipped.")
        return

    start = time.time()
    while vehicle.mode.name != mode_name:
        if time.time() - start > timeout:
            raise TimeoutError(f"Timed out waiting for mode {mode_name}")
        print(f"[DRONE] Waiting for {mode_name} mode...")
        time.sleep(1)


def arm_and_takeoff(target_altitude: float):
    """Arm the drone and take off to target altitude."""
    if SIMULATION_MODE:
        print(f"[SIMULATION] Arm and takeoff to {target_altitude}m")
        return

    connect_vehicle()

    print("[DRONE] Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    wait_until_mode("GUIDED")

    print("[DRONE] Arming motors...")
    vehicle.armed = True

    start = time.time()
    while not vehicle.armed:
        if time.time() - start > 30:
            raise TimeoutError("Timed out while arming motors.")
        print("[DRONE] Waiting for arming...")
        time.sleep(1)

    print(f"[DRONE] Taking off to {target_altitude}m...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"[DRONE] Altitude: {alt:.2f}m")

        if alt >= target_altitude * 0.95:
            print("[DRONE] Target altitude reached.")
            break

        time.sleep(1)


def send_velocity(vx: float, vy: float, vz: float, duration: float):
    """
    Send velocity command in BODY_NED frame.

    vx = forward/backward
    vy = right/left
    vz = down/up in NED frame, so negative means up
    """
    if SIMULATION_MODE:
        print(f"[SIMULATION] Velocity vx={vx}, vy={vy}, vz={vz}, duration={duration}s")
        time.sleep(duration)
        return

    connect_vehicle()

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0,
    )

    start_time = time.time()
    while time.time() - start_time < duration:
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)


def stop_drone(duration: float = 1.0):
    """Stop drone movement and hover."""
    print("[DRONE] Stop / hover")
    send_velocity(0, 0, 0, duration)


def set_yaw(angle: float, relative: bool = True, direction: int = 1):
    """
    Rotate drone by yaw angle.

    direction:
    1  = clockwise/right
    -1 = counter-clockwise/left
    """
    if SIMULATION_MODE:
        side = "right" if direction == 1 else "left"
        print(f"[SIMULATION] Yaw {angle} degrees {side}")
        time.sleep(max(1.0, angle / max(YAW_RATE_DEG_PER_SEC, 1)))
        return

    connect_vehicle()

    is_relative = 1 if relative else 0

    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle,
        YAW_RATE_DEG_PER_SEC,
        direction,
        is_relative,
        0,
        0,
        0,
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

    # Give yaw enough time to complete.
    time.sleep(max(1.0, angle / max(YAW_RATE_DEG_PER_SEC, 1)))


def stop_yaw():
    """Stop yaw movement."""
    print("[DRONE] Stop yaw")
    stop_drone(0.5)


def turn_left_in_place():
    print("[ACTION] Turn left")
    set_yaw(90, relative=True, direction=-1)
    stop_yaw()


def turn_right_in_place():
    print("[ACTION] Turn right")
    set_yaw(90, relative=True, direction=1)
    stop_yaw()


def move_forward_one_grid_step():
    print("[ACTION] Move forward one grid step")
    send_velocity(FORWARD_SPEED_MPS, 0, 0, GRID_STEP_DURATION_SECONDS)
    stop_drone(0.5)


def land_drone():
    """Land the drone."""
    if SIMULATION_MODE:
        print("[SIMULATION] Landing drone.")
        return

    connect_vehicle()

    print("[DRONE] Switching to LAND mode...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f"[DRONE] Landing... altitude={alt:.2f}m")
        time.sleep(1)

    print("[DRONE] Landed successfully.")
    close_vehicle()


# ============================================================
# PATH LOGIC
# ============================================================

def direction_between(current_location: Tuple[int, int], target_location: Tuple[int, int]) -> List[int]:
    """
    Convert target grid movement into a list of step directions.
    If target is diagonal or multiple cells away, it moves rows first, then columns.

    Grid:
    row increases = DOWN
    row decreases = UP
    column increases = RIGHT
    column decreases = LEFT
    """
    current_row, current_col = current_location
    target_row, target_col = target_location

    directions = []

    if target_row > current_row:
        directions.extend([DOWN] * (target_row - current_row))
    elif target_row < current_row:
        directions.extend([UP] * (current_row - target_row))

    if target_col > current_col:
        directions.extend([RIGHT] * (target_col - current_col))
    elif target_col < current_col:
        directions.extend([LEFT] * (current_col - target_col))

    return directions


def turn_to_direction(current_direction: int, target_direction: int) -> int:
    """
    Turn from current direction to target direction.
    Clockwise order: UP -> RIGHT -> DOWN -> LEFT
    """
    if current_direction == target_direction:
        print(f"[ACTION] Already facing {DIRECTION_NAMES[target_direction]}")
        return current_direction

    clockwise_order = [UP, RIGHT, DOWN, LEFT]

    current_index = clockwise_order.index(current_direction)
    target_index = clockwise_order.index(target_direction)

    diff = (target_index - current_index) % 4

    print(
        f"[ACTION] Turn from {DIRECTION_NAMES[current_direction]} "
        f"to {DIRECTION_NAMES[target_direction]}"
    )

    if diff == 1:
        turn_right_in_place()
    elif diff == 2:
        turn_right_in_place()
        turn_right_in_place()
    elif diff == 3:
        turn_left_in_place()

    return target_direction


def apply_direction_to_location(location: Tuple[int, int], direction: int) -> Tuple[int, int]:
    row, col = location

    if direction == UP:
        return row - 1, col
    if direction == DOWN:
        return row + 1, col
    if direction == LEFT:
        return row, col - 1
    if direction == RIGHT:
        return row, col + 1

    raise ValueError(f"Invalid direction: {direction}")


def validate_request_data(data):
    if not data:
        return False, "JSON body is required."

    if "path" not in data:
        return False, "'path' is required."

    if "hover_time_list" not in data:
        return False, "'hover_time_list' is required."

    path = data["path"]
    hover_time_list = data["hover_time_list"]

    if not isinstance(path, list) or len(path) == 0:
        return False, "'path' must be a non-empty list."

    if not isinstance(hover_time_list, list):
        return False, "'hover_time_list' must be a list."

    if len(hover_time_list) < len(path):
        return False, "'hover_time_list' must have at least same length as 'path'."

    for point in path:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or not isinstance(point[0], int)
            or not isinstance(point[1], int)
        ):
            return False, "Each path point must be [row, column], for example [0, 1]."

    for hover_time in hover_time_list:
        if not isinstance(hover_time, (int, float)) or hover_time < 0:
            return False, "Each hover time must be a positive number."

    return True, "OK"


def execute_path(path: List[List[int]], hover_time_list: List[float]):
    """
    Execute full path:
    1. Connect
    2. Takeoff
    3. Hover at starting point
    4. Move point by point
    5. Land
    """
    location = tuple(path[0])
    remaining_points = [tuple(point) for point in path[1:]]

    # Change this if your drone physically starts facing another direction.
    current_direction = DOWN

    connect_vehicle()
    arm_and_takeoff(TAKEOFF_ALTITUDE_METERS)

    print(f"[PATH] Start location: {location}")
    print(f"[DRONE] Initial hover for {hover_time_list[0]} seconds")
    time.sleep(hover_time_list[0])

    for index, target_location in enumerate(remaining_points, start=1):
        print("\n" + "=" * 60)
        print(f"[PATH] Moving to point {index}: {target_location}")
        print(f"[PATH] Current location before moving: {location}")

        directions = direction_between(location, target_location)

        if not directions:
            print("[PATH] Same point. Hover only.")
            time.sleep(hover_time_list[index])
            continue

        for direction in directions:
            current_direction = turn_to_direction(current_direction, direction)
            move_forward_one_grid_step()
            location = apply_direction_to_location(location, direction)
            print(f"[PATH] Updated location: {location}")

        hover_time = hover_time_list[index]
        print(f"[DRONE] Hovering at {location} for {hover_time} seconds")
        time.sleep(hover_time)

    print("\n[PATH] Finished all points.")
    land_drone()


# ============================================================
# FLASK API
# ============================================================

@app.route("/", methods=["GET"])
def home():
    return jsonify({
        "message": "Drone Path API is running",
        "simulation_mode": SIMULATION_MODE,
        "connection_string": CONNECTION_STRING,
        "hardware": "Pixhawk 2.4.8",
        "usage": {
            "method": "POST",
            "url": "/path",
            "json": {
                "path": [[0, 0], [1, 0], [1, 1], [2, 1]],
                "hover_time_list": [2, 3, 2, 4]
            }
        }
    })


@app.route("/status", methods=["GET"])
def status():
    if SIMULATION_MODE:
        return jsonify({
            "mode": "simulation",
            "connected": False,
            "hardware": "Pixhawk 2.4.8",
            "message": "Internal simulation mode enabled. No real drone connected."
        })

    try:
        connect_vehicle()
        return jsonify({
            "mode": "real",
            "connected": True,
            "hardware": "Pixhawk 2.4.8",
            "battery": vehicle.battery.level,
            "armed": vehicle.armed,
            "flight_mode": vehicle.mode.name,
            "altitude": vehicle.location.global_relative_frame.alt
        })
    except Exception as error:
        return jsonify({
            "mode": "real",
            "connected": False,
            "hardware": "Pixhawk 2.4.8",
            "error": str(error)
        }), 500


@app.route("/path", methods=["POST"])
def receive_path():
    data = request.get_json()

    is_valid, message = validate_request_data(data)
    if not is_valid:
        return jsonify({"error": message}), 400

    path = data["path"]
    hover_time_list = data["hover_time_list"]

    print("[API] Received path:", path)
    print("[API] Received hover_time_list:", hover_time_list)

    try:
        execute_path(path, hover_time_list)
    except Exception as error:
        print("[ERROR]", str(error))

        try:
            stop_drone(1)
            land_drone()
        except Exception as landing_error:
            print("[LANDING ERROR]", str(landing_error))

        return jsonify({
            "message": "Failed to execute path",
            "error": str(error)
        }), 500

    return jsonify({
        "message": "Path executed successfully",
        "path": path,
        "hover_time_list": hover_time_list,
        "simulation_mode": SIMULATION_MODE
    }), 200


@app.route("/land", methods=["POST"])
def emergency_land():
    try:
        land_drone()
        return jsonify({"message": "Landing command sent."}), 200
    except Exception as error:
        return jsonify({"error": str(error)}), 500


if __name__ == "__main__":
    print("=" * 60)
    print("Drone Path API")
    print("Hardware: Pixhawk 2.4.8")
    print(f"SIMULATION_MODE={SIMULATION_MODE}")
    print(f"DRONE_CONNECTION={CONNECTION_STRING}")
    print(f"DRONE_BAUD={BAUD_RATE}")
    print("=" * 60)

    app.run(host="0.0.0.0", port=5000, debug=True)
