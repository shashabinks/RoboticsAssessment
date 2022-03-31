
from motor_controller import *
from positioning_controller import *
from cartesian import *

TANGENSIAL_SPEED = 0.12874

ROBOT_ROTATIONAL_SPEED = 0.772881647

ROBOT_ANGULAR_SPEED_IN_DEGREES = 278.237392796


class Epuck_move:
    def __init__(self, robot):
        self.robot = robot
        self.motor_controller = Motor_Controller(self.robot)
        self.positioning_controll = Positioning_controller(self.robot)
        self.destination_set = False

        self.rotation_done = False
        self.forward_done = False

        self.rotation_duration = None
        self.forward_duration = None

        self.rotation_start = None
        self.forward_start = None

        self.rotation_theta = None



    def rotateHeading(self, thetaDot):

        if(not(cartesianIsThetaEqual(thetaDot,0))):

            if self.rotation_duration == None:
                self.rotation_duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES
                self.rotation_start = self.robot.getTime()

            if self.robot.getTime() > self.rotation_start + self.rotation_duration: 
                self.rotation_done = True
                return

            print("currentTime: " + str(self.robot.getTime()), " theta: " + str(thetaDot))

            if thetaDot > 0: self.motor_controller.motorRotateLeft()
            elif thetaDot < 0: self.motor_controller.motorRotateRight()
        
        else:
            self.rotation_done = True
            

    def moveForward(self, distance):

        if self.forward_duration == None:
            self.forward_duration = distance/TANGENSIAL_SPEED
            self.forward_start = self.robot.getTime()

        if self.robot.getTime() > self.forward_start + self.forward_duration:
            self.motor_controller.motorStop()
            self.forward_done = True
            return

        print("currentTime: " + str(self.robot.getTime()), " start + duration: " + str(self.forward_start + self.forward_duration) + " forward: " + str(self.forward_duration))
        self.motor_controller.motorMoveForward()


    def moveToDestination(self, destinationCoordinate):
        
        if self.destination_set == False:
            self.destination_set = True
            
            

        currentCoordinate = self.positioning_controll.positioningControllerGetRobotCoordinate()

        
        if cartesianIsCoordinateEqual(currentCoordinate, destinationCoordinate):
            self.destination_set = False
            self.rotation_done = False
            self.forward_done = False

            self.rotation_duration = None
            self.forward_duration = None
            self.rotation_theta = None
            return True

        

        if not self.rotation_done:
            thetadot = self.positioning_controll.positioningControllerCalcThetaDotToDestination(destinationCoordinate)
            self.rotateHeading(thetadot)


    

        if not self.forward_done and self.rotation_done:
            distanceToDestination = self.positioning_controll.positioningControllerCalcDistanceToDestination(destinationCoordinate)
            self.moveForward(distanceToDestination)


        return False


"""if __name__ == "__main__":
    init_robot()
    destinationCoordinate = ()

    moveToDestination(destinationCoordinate)"""

    
    