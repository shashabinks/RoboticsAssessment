"""bot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from webots.controller import Supervisor, Emitter
import struct



num_epucks = 2

start_cord = (-1.38, -0.875)
epucks = []



# create the Robot instance.
supervisor = Supervisor()

emitter = Emitter(name="robot")


for i in range(num_epucks):
    epucks.append(supervisor.getFromDef(f"EPUCK{i}"))

emitter.setChannel(channel=-1)
emitter.setRange(range=-1)

epucks[0].getField("translation").setSFVec3f([start_cord[0], start_cord[1], 0])
epucks[1].getField("translation").setSFVec3f([start_cord[0] + (6 * 0.25), start_cord[1] + (3 * 0.25), 0])

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# 1. ID, 2. instruction 3. optional: array of cords 
message0 = struct.pack("chd", "0", "move", "[1, 2]")
emitter.send(message0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
