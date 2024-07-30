# Quadcopter-Autonomous-Landing

This is a simulation of autonomous landing system for a UAV on a slow-moving target, handling target search, heading angle estimation, target state estimation, and velocity control without human intervention or communication with the landing marker. The simulation was developed using _CoppeliaSim_ and _Python_. The UAV and landing marker operate independently. This approach ensures the UAV can land accurately even on a moving target.

In order to run the simulation, install [CoppeliaSim EDU](https://www.coppeliarobotics.com/).  CoppeliaSim is a robot simulator software developed by Coppelia Robotics. It is freely available for academic use.

Open your terminal and install the following libraries : 

`pip install cbor2 numpy opencv-python scikit-image pykalman`

Ensure that both the _python_ code folder and the _CoppeliaSim_ folder are in the SAME folder. <br>
Now, open the test.py file and the UAV landing model.ttt file. Run the test.py file. The simulation should start running automatically. 

Pictures from the simulation - 

<figure>
  <img src = "https://i.postimg.cc/nh4yYVVV/Screenshot-2024-07-09-105650.png" width = 500 alt = "Simulation Env" > 
  <span> Simulation Environment </span>
</figure>
  <br><br>

<figure>
  <img src = "https://i.postimg.cc/15qhXcLq/Screenshot-2024-07-11-153732.png" width = 500 alt = "Quadcopter" > 
  <span> Quadcopter with Home position </span>
</figure>
  <br><br>

<figure>
  <img src = "https://i.postimg.cc/nrNyTCQr/Marker-detection.png" width = 500 alt = "Landing Marker Detected" > 
  <span> Landing marker detected using opencv </span>
</figure>
  <br><br>

<figure>
  <img src = "https://i.postimg.cc/PJLGmmJ0/Coordinate-Transformation.png" width = 500 alt = "Coordinate Transformation" > 
  <span> Coordinate transformation from image plane to world frame </span>
</figure>
  <br><br>

<figure>
  <img src = "https://i.postimg.cc/h4wRK33n/Screenshot-2024-07-09-114019.png" width = 500 alt = "Approach" > 
  <span> Approaching landing marker</span>
</figure>
<br><br>
