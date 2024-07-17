import numpy as np
import math
from collections import namedtuple
import random

# Define the PATH_TYPE data structure
PathPoint = namedtuple('PathPoint', ['x', 'y', 'heading', 'curvature', 'distance'])

# Define a function to calculate the distance between two points
def distance(point1, point2):
    return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

# Define the vehicle class
class Vehicle:
    def __init__(self, x, y, heading, speed):
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = speed
    
    def update_position(self, dx, dy, dheading, dspeed):
        self.x += dx
        self.y += dy
        self.heading += dheading
        self.speed += dspeed
    
    def current_position(self):
        return PathPoint(self.x, self.y, self.heading, 0, 0)

# Define the path class
class Path:
    def __init__(self):
        self.points = []
    
    def add_point(self, x, y, heading, curvature, distance):
        self.points.append(PathPoint(x, y, heading, curvature, distance))
    
    def closest_point(self, vehicle_position):
        closest_point = min(self.points, key=lambda point: distance(point, vehicle_position))
        return closest_point
    
    def goal_point(self, vehicle_position, lookahead_distance):
        for point in self.points:
            if distance(vehicle_position, point) >= lookahead_distance:
                return point
        return self.points[-1]  # If no point is found within lookahead distance, return the last point

# Define the pure pursuit algorithm
def pure_pursuit(vehicle, path, lookahead_distance):
    # Step 1: Determine the current location of the vehicle
    current_position = vehicle.current_position()
    
    # Step 2: Find the path point closest to the vehicle
    closest_point = path.closest_point(current_position)
    
    # Step 3: Find the goal point
    goal_point = path.goal_point(current_position, lookahead_distance)
    
    # Step 4: Transform the goal point to vehicle coordinates
    dx = goal_point.x - vehicle.x
    dy = goal_point.y - vehicle.y
    local_goal_x = dx * math.cos(vehicle.heading) + dy * math.sin(vehicle.heading)
    local_goal_y = -dx * math.sin(vehicle.heading) + dy * math.cos(vehicle.heading)
    
    # Step 5: Calculate the curvature
    if local_goal_x != 0:
        curvature = 2 * local_goal_y / (local_goal_x**2 + local_goal_y**2)
    else:
        curvature = 0
    
    # Step 6: Update the vehicle’s position (simulating the effect of the command)
    # For simplicity, assume a small time step
    dt = 0.1
    dx = vehicle.speed * math.cos(vehicle.heading) * dt
    dy = vehicle.speed * math.sin(vehicle.heading) * dt
    dheading = curvature * vehicle.speed * dt
    dspeed = 0  # No change in speed for simplicity
    
    vehicle.update_position(dx, dy, dheading, dspeed)
    
    return curvature

# Function to estimate the state of the moving target
def estimate_target_state(target, dt):
    target.x += target.speed * math.cos(target.heading) * dt
    target.y += target.speed * math.sin(target.heading) * dt
    return target

# Example usage
if __name__ == "__main__":
    # Initialize vehicle and path
    vehicle = Vehicle(0, 0, 0, 1.0)
    path = Path()
    lookahead_distance = 1.5
    
    # Create a sample path (e.g., a straight line for simplicity)
    for i in range(10):
        path.add_point(i, i, 0, 0, i)
    
    # Simulate the moving target
    target = Vehicle(10, 10, 0, 0.5)
    
    # Run the simulation
    for _ in range(50):
        # Estimate target state
        dt = 0.1
        target = estimate_target_state(target, dt)
        
        # Update path to follow the moving target
        path.add_point(target.x, target.y, 0, 0, distance(vehicle.current_position(), target.current_position()))
        
        # Run the pure pursuit algorithm
        curvature = pure_pursuit(vehicle, path, lookahead_distance)
        print(f"Vehicle position: {vehicle.current_position()}, Curvature: {curvature:.4f}, Target position: {target.current_position()}")

