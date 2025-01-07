class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.Visited = False
        self.Occupied = False
        self.Neighbors = []

    def __repr__(self):
        return f"Cell({self.x}, {self.y})"

def create_map(map_array):
    rows = len(map_array)
    cols = len(map_array[0])
    grid = [[Cell(x, y) for y in range(cols)] for x in range(rows)]

    for x in range(rows):
        for y in range(cols):
            if map_array[x][y] == 'X':
                grid[x][y].Occupied = True
                start = grid[x][y]
            elif map_array[x][y] == 'O':
                grid[x][y].Occupied = False

            # Set neighbors in counter-clockwise order starting from the right
            if y + 1 < cols:
                grid[x][y].Neighbors.append(('right', grid[x][y + 1]))
            if x + 1 < rows:
                grid[x][y].Neighbors.append(('down', grid[x + 1][y]))
            if y - 1 >= 0:
                grid[x][y].Neighbors.append(('left', grid[x][y - 1]))
            if x - 1 >= 0:
                grid[x][y].Neighbors.append(('up', grid[x - 1][y]))

    return grid, start

def checkForUnvisitedNeighbors(cell):
    for direction, neighbor in cell.Neighbors:
        if not neighbor.Visited:
            return True
    return False

def traversal(currentSpace, visited, directions):
    while checkForUnvisitedNeighbors(currentSpace):
        for direction, neighbor in currentSpace.Neighbors:
            if not neighbor.Visited:
                currentSpace = neighbor
                directions.append(direction)
                break
        currentSpace.Visited = True
        visited.append(currentSpace)

def BCD(start):
    visited = []
    directions = []
    visited.append(start)
    start.Visited = True
    currentSpace = start

    traversal(currentSpace, visited, directions)

    for cell in reversed(visited):
        if checkForUnvisitedNeighbors(cell):
            traversal(cell, visited, directions)

    return visited, directions

def print_map(grid, path):
    map_output = [['O' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    for cell in path:
        map_output[cell.x][cell.y] = 'X'
    for row in map_output:
        print(' '.join(row))
    print()

# Initial map
map_array = [
    ['O', 'O', 'O', 'O', 'O'],
    ['O', 'O', 'O', 'O', 'O'],
    ['O', 'O', 'O', 'O', 'O'],
    ['O', 'O', 'O', 'O', 'O'],
    ['X', 'O', 'O', 'O', 'O']
]

grid, start = create_map(map_array)
path, directions = BCD(start)

# Print the final path
#print("Final Path:")
#print_map(grid, path)

# List the path coordinates
#print("List of Path Coordinates:")
#for cell in path:
#    print((cell.x, cell.y))

# List the directions
#print("List of Path Directions:")
#print(directions)









# sim control code starts here ---------------------------------

"""tractor_bcd_algorithm controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from math import pi, sin
import math
from vehicle import Driver
import time


driver = Driver()
# create the Robot instance.
#robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

tractor_orientation = "north"

#turn robot 90 degrees counter-clockwise
def right_turn():
    i = 0
    while i<270:
        driver.setCruisingSpeed(1)
        driver.setSteeringAngle(1)
        driver.step()
        i += 1
    
      
#turn robot 90 degrees clockwise  
def left_turn():
    i = 0
    while i<270:
        driver.setCruisingSpeed(1)
        driver.setSteeringAngle(-1)
        driver.step()
        i += 1

def forward_step():
    i = 0
    while i<130:
        driver.setCruisingSpeed(5)
        driver.setSteeringAngle(0)
        driver.step()
        i += 1

def followPath(directions):
    i = 0
    tractor_orientation = "north"
    while i<len(directions):
        if directions[i] == 'up':
            if tractor_orientation == "east":
                left_turn()
                tractor_orientation = "north"
            elif tractor_orientation == "west":
                right_turn()
                tractor_orientation = "north"
            elif tractor_orientation == "south":
                right_turn()
                right_turn()
                tractor_orientation = "north"
            else:
                forward_step()
            i += 1
        
        elif directions[i] == 'right':
            if tractor_orientation == "north":
                right_turn()
                tractor_orientation = "east"
            elif tractor_orientation == "west":
                right_turn()
                right_turn()
                tractor_orientation = "east"
            elif tractor_orientation == "south":
                left_turn()
                tractor_orientation = "east"
            else:
                forward_step()
                
            i += 1
        
        elif directions[i] == 'left':
            if tractor_orientation == "north":
                left_turn()
                tractor_orientation = "west"
            elif tractor_orientation == "east":
                right_turn()
                right_turn()
                tractor_orientation = "west"
            elif tractor_orientation == "south":
                right_turn()
                tractor_orientation = "west"
            else:
                forward_step()
                
            i += 1

        else:
            if tractor_orientation == "north":
                right_turn()
                right_turn()
                tractor_orientation = "south"
            elif tractor_orientation == "west":
                left_turn()
                tractor_orientation = "south"
            elif tractor_orientation == "east":
                right_turn()
                tractor_orientation = "south"
            else:
                forward_step()
                
            i += 1


followPath(directions)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
#while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
#    pass

# Enter here exit cleanup code.
