from json.encoder import INFINITY
from webots.controller import Robot, Motor, Receiver

MAX_SPEED = 6.28

class Motor_Controller:
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        Motor.setPosition(self.left_motor, INFINITY)
        Motor.setPosition(self.right_motor, INFINITY)
        Motor.setVelocity(self.left_motor, 0.0)
        Motor.setVelocity(self.right_motor, 0.0)

        
    def motorStop(self):
        Motor.setVelocity(self.left_motor, 0.0)
        Motor.setVelocity(self.right_motor, 0.0)


    def motorMoveForward(self):
        Motor.setVelocity(self.left_motor, MAX_SPEED)
        Motor.setVelocity(self.right_motor, MAX_SPEED)

    def motorRotateLeft(self):
        Motor.setVelocity(self.left_motor, -MAX_SPEED)
        Motor.setVelocity(self.right_motor, MAX_SPEED)

    def motorRotateRight(self):
        Motor.setVelocity(self.left_motor, MAX_SPEED)
        Motor.setVelocity(self.right_motor, -MAX_SPEED)
