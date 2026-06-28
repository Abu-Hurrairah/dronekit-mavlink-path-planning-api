from flask import Flask, request, jsonify
import time

app = Flask(__name__)

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


def arm_and_takeoff(target_altitude: float):
    print(f"[SIMULATION] Arm and takeoff to {target_altitude}m")


def send_velocity(vx: float, vy: float, vz: float, duration: float):
    print(f"[SIMULATION] Velocity vx={vx}, vy={vy}, vz={vz}, duration={duration}s")
    time.sleep(duration)


def stop_drone(duration: float = 1.0):
    print("[SIMULATION] Stop / hover")
    send_velocity(0, 0, 0, duration)


def set_yaw(angle: float, relative: bool = True, direction: int = 1):
    side = "right" if direction == 1 else "left"
    print(f"[SIMULATION] Yaw {angle} degrees {side}")
    time.sleep(1)


def turn_left_in_place():
    print("[ACTION] Turn left")
    set_yaw(90, relative=True, direction=-1)
    stop_drone(0.5)


def turn_right_in_place():
    print("[ACTION] Turn right")
    set_yaw(90, relative=True, direction=1)
    stop_drone(0.5)


def move_forward_one_grid_step():
    print("[ACTION] Move forward one grid step")
    send_velocity(0.25, 0, 0, 3)
    stop_drone(0.5)


def land_drone():
    print("[SIMULATION] Landing drone.")


def direction_between(current_location, target_location):
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


def turn_to_direction(current_direction, target_direction):
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


def apply_direction_to_location(location, direction):
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


def execute_path(path, hover_time_list):
    location = tuple(path[0])
    remaining_points = [tuple(point) for point in path[1:]]
    current_direction = DOWN

    arm_and_takeoff(1)

    print(f"[PATH] Start location: {location}")
    print(f"[SIMULATION] Initial hover for {hover_time_list[0]} seconds")
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
        print(f"[SIMULATION] Hovering at {location} for {hover_time} seconds")
        time.sleep(hover_time)

    print("\n[PATH] Finished all points.")
    land_drone()


@app.route("/", methods=["GET"])
def home():
    return jsonify({
        "message": "Drone Path API simulation is running",
        "usage": {
            "method": "POST",
            "url": "/path",
            "json": {
                "path": [[0, 0], [1, 0], [1, 1], [2, 1]],
                "hover_time_list": [2, 3, 2, 4]
            }
        }
    })


@app.route("/path", methods=["POST"])
def receive_path():
    data = request.get_json()

    is_valid, message = validate_request_data(data)
    if not is_valid:
        return jsonify({"error": message}), 400

    path = data["path"]
    hover_time_list = data["hover_time_list"]

    try:
        execute_path(path, hover_time_list)
    except Exception as error:
        return jsonify({
            "message": "Failed to execute path",
            "error": str(error)
        }), 500

    return jsonify({
        "message": "Path executed successfully in simulation mode",
        "path": path,
        "hover_time_list": hover_time_list
    }), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
