# Webots
The aim of the project is to design and create a robot using Webots software and to enable the robot to navigate automatically. The first goal is that the robot can successfully escape a maze that it has already seen; the second goal is that the robot can successfully escape a maze that it has not seen; and the third goal is that the robot can still successfully escape by adding moving obstacles to the maze based on the second goal. The robot will have two minutes to escape the maze and the performance will be judged on how far it gets from the target and how many collisions it has with the environment or obstacles during the two minutes. 
This project report is a description of the project realization process, from the design of the robot, the creation of the robot, and the navigation strategies and algorithms used for each of the three objectives.

1. Build the model of robot:
FourWheelsRobot.proto is the first version of the robot
FourWheelsRound.proto is the secon d version of the robot

2. Controller:
A number of parameters need to be set prior to the navigation strategy, such as the parameters for the wheels when the robot turns slowly to the left or right; the parameters for the wheels when the robot moves forward and stops and the parameters for the wheels when the robot turns straight in place to the left or right (hard turn). 
It is also necessary to read the data from the GPS on the robot, as the GPS sensors are located on both sides of the robot, it is possible to calculate the robot position. Simply divide the GPS readings by two to obtain the robot's centre position.
Equations:

avg_x = (gps_value[0][0] + gps_value[1][0]) / 2
avg_y = (gps_value[0][1] + gps_value[1][1]) / 2
robot_location[0] = avg_x
robot_location[1] = avg_y

The get target position function is to calculate the distance and direction of the target position relative to the robot's own position.
The move to goal function is to the set threshold of angle delta theta, if the angle between the position of the robot and the position of the goal is between -2 to 2 then the robot will move forward to the goal, if the angle is < -2, the robot will turn left to reach the goal; if the angle is > 2, the robot will turn right to reach the goal

3. Navigation of seen and unseen maze:
The robot escapes from seen and unseen mazes using the same navigation algorithm - A* path planning. both seen and new mazes are two-dimensional occupancy grids, and A* path planning determines the route to be taken by analysing these grids and the coordinates of the start and end points.
In the Occupancy Grid of a maze, all grids are labeled 1 or 0, where 1 represents walls and occluders in the maze and 0 represents positions that can be passed through. The position of the start and end points will be represented by the form of point (x, y) in the coordinate axis.

4. Navigation of unseen maze with moving obstacles:
Distance sensors R2, mid and L2 are used to detect obstacles in front of the robot, with a total field of view of 60 degrees, each 30 degrees apart. If R2 detects an obstacle it indicates that there is an obstacle on the front-right of the robot – therefore it will trigger the obstacle detection protocol. In this case it will make hard left turns. Similar logic is applied to the L2 sensor, the robot will make hard right turns if L2 detects an obstacle. The mid sensor is also responsible for detected obstacles in front, and it was decided to turn left if mid sensor is triggered.

5. The final display in Webots: KnownMaze.wbt


