## SLAM R1D4 autonomous 2D Lidar path finding project


A python-based 2D robotics simulation where an autonomous robot explores an unknown indoor environment using a simulated Lidar sensor. The robot will map the environment, navigator through obstacles using adaptive clustering to then work its way towards a goal. 


### Features

- **Simulated Lidar sensor**: Casts rays to detect obstacles in a 2D image map.

- **Adaptive Point clustering**: Groups lidar points into distinct obstacles based of their angle and distance. 


- **Dynamic graph generation**: Identifies gaps between obstacles using their extremeties to then dynamically generate new nodes.

- **Memory and loop prevention**: Keeps track of the path and generates new nodes to prevent the construction of an infinite trees/re-exploring the same area.

- **Pygam Visualization**: Real-time visualization of the robot moving using the simulated LidarPoints and the internal memory map. 


### How to run

1. Launch the pygame simulation by running `path_find_display.py`

2. Control the simulation by pressing SPACE to calculate the next path segment so the robot moves forward. 

3. At the end of task 2, the robot will move to the new point. Repeat it until the robot has reached the goal.

4. Press close the window to exit. 

*(Upon reaching the goal, the script will automatically save a screenshot of the final explored map as final_exploration_map.png)*


### Custom maps


You can create your own environments! Just draw a 2D map with black lines for the walls/obstacles and white for the empty spaces, save it as a `.png`, and update the `mapImg_path` and `mapDimension` variables in `path_finding_displya.py`. 


