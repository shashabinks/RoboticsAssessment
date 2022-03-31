from json.encoder import INFINITY
from epuck_controller import robot
from webots.controller import Robot, Motor, Receiver

MAX_SPEED = 6.28

left_motor, right_motor = None, None

def motorControllerInit(time_step):

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    Motor.setPosition(left_motor, INFINITY)
    Motor.setPosition(right_motor, INFINITY)
    Motor.setVelocity(left_motor, 0.0)
    Motor.setVelocity(right_motor, 0.0)

    
def motorStop():
    Motor.setVelocity(left_motor, 0.0)
    Motor.setVelocity(right_motor, 0.0)


def motorMoveForward():
    Motor.setVelocity(left_motor, MAX_SPEED)
    Motor.setVelocity(right_motor, MAX_SPEED)

def motorRotateLeft():
    Motor.setVelocity(left_motor, -MAX_SPEED)
    Motor.setVelocity(right_motor, MAX_SPEED)

def motorRotateRight():
    Motor.setVelocity(left_motor, MAX_SPEED)
    Motor.setVelocity(right_motor, -MAX_SPEED)
