import math
import numpy as np

class PathPoint:
    def __init__(self, x, y, heading, curvature, distance):
        self.x = x
        self.y = y
        self.heading = heading
        self.curvature = curvature
        self.distance = distance

class Position:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

def get_initial_position():
    # Placeholder function to simulate getting the initial position of the vehicle
    return Position(0, 0, 0)  # Example initial position

def find_closest_path_point(path, current_pos):
    closest_point = None
    min_dist = float('inf')
    for point in path:
        dist = math.sqrt((point.x - current_pos.x)**2 + (point.y - current_pos.y)**2)
        if dist < min_dist:
            min_dist = dist
            closest_point = point
    return closest_point

def find_goal_point(path, current_pos, lookahead_distance):
    for point in path:
        dist = math.sqrt((point.x - current_pos.x)**2 + (point.y - current_pos.y)**2)
        if dist >= lookahead_distance:
            return point
    return path[-1]  # If no point is found, return the last point

def transform_to_vehicle_coordinates(goal_point, current_pos):
    dx = goal_point.x - current_pos.x
    dy = goal_point.y - current_pos.y
    angle = current_pos.heading
    local_x = math.cos(angle) * dx + math.sin(angle) * dy
    local_y = -math.sin(angle) * dx + math.cos(angle) * dy
    return local_x, local_y

def calculate_curvature(local_x, local_y):
    if local_x == 0:
        return 0
    return 2 * local_y / (local_x**2 + local_y**2)

def update_vehicle_position(current_pos, steering_angle, distance):
    # This function simulates the effect of the steering command on the vehicle's position
    current_pos.x += distance * math.cos(current_pos.heading)
    current_pos.y += distance * math.sin(current_pos.heading)
    current_pos.heading += steering_angle  # Simplified model

import time

# Define the path
path = [PathPoint(x, y, 0, 0, 0) for x, y in zip(np.linspace(0, 100, 100), np.linspace(0, 100, 100))]

# Main loop
def main():
    current_pos = get_initial_position()
    lookahead_distance = 10.0  # Example lookahead distance

    while True:
        # Step 1: Determine the current location of the vehicle
        # No need to call get_current_position() again; use the updated current_pos
        print(f"Current Position: x={current_pos.x}, y={current_pos.y}, heading={current_pos.heading}")

        # Step 2: Find the path point closest to the vehicle
        closest_point = find_closest_path_point(path, current_pos)
        print(f"Closest Point: x={closest_point.x}, y={closest_point.y}")

        # Step 3: Find the goal point
        goal_point = find_goal_point(path, current_pos, lookahead_distance)
        print(f"Goal Point: x={goal_point.x}, y={goal_point.y}")

        # Step 4: Transform the goal point to vehicle coordinates
        local_x, local_y = transform_to_vehicle_coordinates(goal_point, current_pos)
        print(f"Local Coordinates: x={local_x}, y={local_y}")

        # Step 5: Calculate the curvature
        curvature = calculate_curvature(local_x, local_y)
        print(f"Curvature: {curvature}")

        # Step 6: Update the vehicle's position (simulation only)
        update_vehicle_position(current_pos, curvature, 1.0)  # Assuming a constant distance increment

        # Print the updated position
        print(f"Updated Position: x={current_pos.x}, y={current_pos.y}, heading={current_pos.heading}\n")

        # Check if the vehicle has reached the end of the path
        if math.sqrt((current_pos.x - path[-1].x)**2 + (current_pos.y - path[-1].y)**2) < 1.0:
            print("Reached the end of the path.")
            break

        # Add a small delay to simulate real-time behavior
        time.sleep(5)

if __name__ == "__main__":
    main()

