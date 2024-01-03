"""Bug1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS
import math
import numpy as np
import time

##############################################################################
# A* pathfinding
##############################################################################
# import packages
##############################################################################
import numpy as np
import heapq
import csv

##############################################################################
# read in csv file
##############################################################################
maze = []

#**************************** MUST CHANGE**************************
og_filename = 'OccupancyGrid.csv'
#***************************************************************************

with open(og_filename, 'r') as csv_file:
    reader = csv.reader(csv_file)
    for row in reader:
        for counter, i in enumerate(row):
            if   i == '1': row[counter] = 1
            elif i == '0': row[counter] = 0
        maze.append(row)

grid = np.array(maze)

# start point and goal
start = (0.5, -0.5)
goal = (3, -4)
start = (int(abs(start[1])*10), int(abs(start[0])*10))
goal = (int(abs(goal[1])*10), int(abs(goal[0])*10))

##############################################################################
# heuristic function for path scoring
##############################################################################
def heuristic(a, b):
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
##############################################################################
# path finding function
##############################################################################
def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}  
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
 
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
 
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

route = astar(grid, start, goal)
route = route + [start]
route = route[::-1]

for count, (i, j) in enumerate(route):
    route[count] = j/10, -i/10

print(route)

###################################################################################
TIME_STEP = 64
robot = Robot()
    
ds = []
dsNames = ['ds_R0', 'ds_R1', 'ds_R2', 'ds_mid', 'ds_L2', 'ds_L1', 'ds_L0']
for i in range(7):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

def turnLeft(wheels):
    wheels[0].setVelocity(-2.0)
    wheels[1].setVelocity(5.78)
    wheels[2].setVelocity(-2.0)
    wheels[3].setVelocity(5.78)

def turnRight(wheels):
    wheels[0].setVelocity(5.78)
    wheels[1].setVelocity(-2.0)
    wheels[2].setVelocity(5.78)
    wheels[3].setVelocity(-2.0)
    
def moveForward(wheels):
    wheels[0].setVelocity(6.78)
    wheels[1].setVelocity(6.78)
    wheels[2].setVelocity(6.78)
    wheels[3].setVelocity(6.78)
    
def stop(wheels):
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(0.0)
    wheels[3].setVelocity(0.0)
    
def hardLeft(wheels):
    wheels[0].setVelocity(-3.0)
    wheels[1].setVelocity(3.0)
    wheels[2].setVelocity(-3.0)
    wheels[3].setVelocity(3.0)
    
def hardRight(wheels):
    wheels[0].setVelocity(3.0)
    wheels[1].setVelocity(-3.0)
    wheels[2].setVelocity(3.0)
    wheels[3].setVelocity(-3.0)

def initGps(TIME_STEP, robot):
    gps = []
    gpsNames = ['gps1', 'gps2']
    value = [[0, 0, 0], [0, 0, 0]]
    for i in range(len(gpsNames)):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)

    return gps, value

def initWheels(robot):
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

    for i in range(len(wheelsNames)):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(6.78)

    return wheels

def getGpsData(gps, value):
    # Put solution to Q2b here
   # print("Getting GPS Data")
    value[0] = gps[0].getValues()
    value[1] = gps[1].getValues()
    #print("GPS Values:", value)
    return value

def getRobotPose(gps_value):
    pose = [0, 0]
    robot_location = [0, 0]
    # Put solution to Q2c here
    avg_x = (gps_value[0][0] + gps_value[1][0]) / 2
    avg_y = (gps_value[0][1] + gps_value[1][1]) / 2
    robot_location[0] = avg_x
    robot_location[1] = avg_y

    orientation_vector = [gps_value[0][1] - gps_value[1][1], -1*(gps_value[0][0] - gps_value[1][0])]
    mag_orientation_vector = math.sqrt(pow(orientation_vector[0],2)+pow(orientation_vector[1],2))

    if mag_orientation_vector != 0:
        pose[0] = orientation_vector[0]/mag_orientation_vector
        pose[1] = orientation_vector[1]/mag_orientation_vector
   # print("Robot location: ", robot_location)
   # print("Robot Pose: ", pose)
    return robot_location, pose

def getTargetPose(goal, robot_location):
    target_pose = [0, 0]
    # Put solution for Q2d here
    target_orientation_vector = [goal[0]-robot_location[0], goal[1]-robot_location[1]]
    mag_target_orientation_vector = math.sqrt(pow(target_orientation_vector[0],2)+pow(target_orientation_vector[1],2))
    if mag_target_orientation_vector != 0:
        target_pose[0] = target_orientation_vector[0]/mag_target_orientation_vector
        target_pose[1] = target_orientation_vector[1]/mag_target_orientation_vector
    
    #print("Target pose:", target_pose)
    return target_pose
    
def moveToGoal(wheels, goal, gpsValue, pose, target_pose, delta_theta, route_idx):
    # Complete in Q2f
    if (-2 < delta_theta < 2) or side_check():
        moveForward(wheels)
        print("Forward")
    elif delta_theta < 0:
        turnLeft(wheels)
        print("Soft turn left")
    elif delta_theta > 0:
        turnRight(wheels)
        print("Soft turn right")
    #print("Delta theta: ", delta_theta)
    print("Moving to goal: ", route[route_idx])
    print("==== UNDER ======")
    

def side_check():
    if (ds[0].getValue() < 650.0 and ds[1].getValue() < 650.0) or (ds[5].getValue() < 650.0 and ds[6].getValue() < 650.0):
        return True

        

def tutorialGpsController(route_idx):
    goal = route[route_idx]
    mode = "MoveToGoal"
    avoidObstacleCounter = 0
    corner = "None"
    obstacle_detected = False
    
    gps, gps_value = initGps(TIME_STEP, robot)
    wheels = initWheels(robot)


    while robot.step(TIME_STEP) != -1:
        gps_value = getGpsData(gps, gps_value)

        robot_location = [0, 0] # Calculate this in Q2c
        robot_location, robot_pose = getRobotPose(gps_value)
        target_pose = getTargetPose(goal, robot_location)
        
        if avoidObstacleCounter > 0:
            obstacle_detected = True
            avoidObstacleCounter -= 1
            if corner == "Right":
                print("CORNER: Hard turn right")
                hardRight(wheels)
            elif corner == "Left":
                print("CORNER: Hard turn left")
                hardLeft(wheels)
            elif ds[3].getValue() < 650.0:
                print("Hard turn left")
                hardLeft(wheels)
            elif ds[4].getValue() < 650.0:
                print("Hard turn right")
                hardRight(wheels)
            continue
            
        elif ds[2].getValue() < 650.0 or ds[3].getValue() < 650.0 or ds[4].getValue() < 650.0:
            if ds[1].getValue() < 950.0 and ds[2].getValue() < 950.0 and ds[3].getValue() < 950.0 and ds[4].getValue() < 950.0 and ds[5].getValue() < 950.0:
                if ds[0].getValue() > ds[6].getValue():
                    corner = "Right"
                else:
                    corner = "Left"
            else:
                corner = "None"
        #Obstacle detected
            avoidObstacleCounter = 15
                      

        delta_theta = math.acos(np.dot(target_pose, robot_pose))*(180/math.pi)
        if math.asin(np.cross(target_pose, robot_pose)) < 0:
            delta_theta *= -1
            
        robo_loc = (round(robot_location[0], 1), round(robot_location[1], 1))
        print(robo_loc)
        print("Moving to goal: ", route[route_idx])
        
        if abs(goal[0]-robot_location[0]) < 0.1 and abs(goal[1]-robot_location[1]) < 0.1:
            if route_idx < len(route)-1:
                route_idx += 1                
                goal = route[route_idx]
                mode = "MoveToGoal"
            elif abs(goal[0]-robot_location[0]) < 0.05 and abs(goal[1]-robot_location[1]) < 0.05:
                print("Goal Reached")
                mode = "GoalReached"
                stop(wheels)

        
        elif mode == "MoveToGoal":
            if obstacle_detected or side_check():
                if robo_loc in route:
                    route_idx = route.index(robo_loc)
                    goal = route[route_idx]
            obstacle_detected = False
            print(obstacle_detected)
            moveToGoal(wheels=wheels, goal=goal, gpsValue=gps_value, pose=robot_pose, target_pose=target_pose, delta_theta=delta_theta, 
                       route_idx=route_idx)
        print()


if __name__ == "__main__":
    print("tutorial-GPS")
    route_idx = 0
    tutorialGpsController(route_idx)
