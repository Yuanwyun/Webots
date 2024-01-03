# Webots
The aim of the project is to design and create a robot using Webots software and to enable the robot to navigate automatically. The first goal is that the robot can successfully escape a maze that it has already seen; the second goal is that the robot can successfully escape a maze that it has not seen; and the third goal is that the robot can still successfully escape by adding moving obstacles to the maze based on the second goal. The robot will have two minutes to escape the maze and the performance will be judged on how far it gets from the target and how many collisions it has with the environment or obstacles during the two minutes. 
This project report is a description of the project realization process, from the design of the robot, the creation of the robot, and the navigation strategies and algorithms used for each of the three objectives.

1. Build the model of robot:
FourWheelsRobot.proto is the first version of the robot
FourWheelsRound.proto is the secon d version of the robot

2. Controller:
A number of parameters need to be set prior to the navigation strategy, such as the parameters for the wheels when the robot turns slowly to the left or right; the parameters for the wheels when the robot moves forward and stops and the parameters for the wheels when the robot turns straight in place to the left or right (hard turn). Figure 5 shows all the wheels parameters under different situations.

