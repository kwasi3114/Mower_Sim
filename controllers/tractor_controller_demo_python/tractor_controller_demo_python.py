"""tractor_controller_demo_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from math import pi, sin
import math
from vehicle import Driver
import time

# create the Robot instance.
#robot = Robot()
#motor = robot.getDevice("my_motor")

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

#F = 2.0   # frequency 2 Hz
#t = 0.0   # elapsed simulation time

driver = Driver()
#driver.setSteeringAngle(0.75)

# Main loop
# - perform simulation steps until Webots is stopping the controller
#while robot.step(timestep) != -1:

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
    while i<80:
        driver.setCruisingSpeed(5)
        driver.setSteeringAngle(0)
        driver.step()
        i += 1

#forward_step()
#right_turn()
while driver.step() != -1:
    #driver.setCruisingSpeed(5)
    #driver.setSteeringAngle(1)
    
    
    right_turn()
    forward_step()
    left_turn()
    forward_step()
    
    #driver.setCruisingSpeed(5)
    #driver.setSteeringAngle(0)
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass
    
    #position = sin(t * 2.0 * pi * F)
    #motor.setPosition(position)
    #t += TIME_STEP / 1000.0

# Enter here exit cleanup code.
