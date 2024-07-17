import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from quadrocoptertrajectory import RapidTrajectory  # Assuming your trajectory code is in a module named `your_trajectory_module`

# Example initial states (replace these with your actual initial states)
initial_pos = np.array([0, 0, 2])
initial_vel = np.array([0, 0, 0])
initial_acc = np.array([0, 0, 0])
gravity = np.array([0, 0, -9.81])


# Initialize the RapidTrajectory class
trajectory = RapidTrajectory(initial_pos, initial_vel, initial_acc, gravity)


# Main control function
def generate(cx : int, cy : int, landingPos : list):
    # Get the current target position from the calc() function
    target_x, target_y = cx, cy
    print(cx, cy, landingPos)
    # Assuming the z position remains constant for simplicity
    target_pos = np.array([target_x, target_y, initial_pos[2]])
    
    # Update the goal position
    trajectory.set_goal_position(target_pos)
    
    # Set the goal velocity and acceleration to zero (assuming you want the drone to stop at the target)
    trajectory.set_goal_velocity(np.array([0, 0, 0]))
    trajectory.set_goal_acceleration(np.array([0, 0, 0]))
    
    # Generate the trajectory with an example timeToGo
    time_to_go = 2  # This should be adjusted based on your requirements
    trajectory.generate(time_to_go)
    
    # Example usage: get the position, velocity, and acceleration at time t
      # Example time
    
    numPlotPoints = 100
    pos = np.zeros((numPlotPoints, 3), dtype=np.int32)
    time = np.linspace(0, time_to_go, numPlotPoints) # generates 100 evenly spaced points between 0 and 2

    for i in range(numPlotPoints):
        t = time[i]
        pos[i, :] = trajectory.get_position(t)

    
    # ax = plt.axes(projection = "3d")
    
    # ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label = "Trajectory")
    # ax.scatter(landingPos[0], landingPos[1], landingPos[2], label = "Target Position")
    # ax.set_title("Trajectory plot")
    # ax.set_xlabel("px")
    # ax.set_ylabel("px")
    # ax.set_zlabel("px")

    # ax.legend()

    # plt.show()
    # Use pos, vel, and acc to control your drone
    # For example, send these values to your drone's control system
    
    # Implement a small delay or a rate to run this loop at a specific frequency
    # For example, if you want to run this loop at 10 Hz, use a delay of 0.1 seconds
  
