from json.encoder import INFINITY
from webots.controller import Robot, Motor, Receiver



class Motor_Controller:
    def __init__(self, robot, rot_speed, forward_speed):
        self.robot = robot
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.rot_speed = rot_speed
        self.forward_speed = forward_speed
        Motor.setPosition(self.left_motor, INFINITY)
        Motor.setPosition(self.right_motor, INFINITY)
        Motor.setVelocity(self.left_motor, 0.0)
        Motor.setVelocity(self.right_motor, 0.0)

        
    def motorStop(self):
        Motor.setVelocity(self.left_motor, 0.0)
        Motor.setVelocity(self.right_motor, 0.0)


    def motorMoveForward(self):
        Motor.setVelocity(self.left_motor, self.forward_speed)
        Motor.setVelocity(self.right_motor, self.forward_speed)

    def motorRotateLeft(self):
        Motor.setVelocity(self.left_motor, -self.rot_speed)
        Motor.setVelocity(self.right_motor, self.rot_speed)

    def motorRotateRight(self):
        Motor.setVelocity(self.left_motor, self.rot_speed)
        Motor.setVelocity(self.right_motor, -self.rot_speed)
