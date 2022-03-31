
from motor_controller import *
from positioning_controller import *
from cartesian import *

TANGENSIAL_SPEED = 0.12874

ROBOT_ROTATIONAL_SPEED = 0.772881647

ROBOT_ANGULAR_SPEED_IN_DEGREES = 278.237392796


class Epuck_move:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = self.getTimeStep()
        self.motor_controller = Motor_Controller(self.robot, self.time_step)
        self.positioning_controll = Positioning_controller(self.robot)


    def getTimeStep(self):
        self.time_step = int(self.robot.getBasicTimeStep())
        return self.time_step

    def step(self):
        if self.robot.step(self.time_step) == -1:
            exit()

    def rotateHeading(self, thetaDot):

        if(not(cartesianIsThetaEqual(thetaDot,0))):

            duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES

            print(f"duration to face the dest: {duration}")

            if thetaDot > 0: self.motor_controller.motorRotateLeft()
            elif thetaDot < 0: self.motor_controller.motorRotateRight()

            start_time = int(self.robot.getBasicTimeStep())
            print("start time: " + str(start_time))
            print("duration: " + str(duration))

            while(int(self.robot.getBasicTimeStep()) < start_time + duration):
                #print(int(self.robot.getBasicTimeStep()))
                self.step()

            self.motor_controller.motorStop()
            

    def moveForward(self, distance):

        duration = distance/TANGENSIAL_SPEED

        self.motor_controller.motorMoveForward()

        start_time = int(self.robot.getBasicTimeStep())

        while(int(self.robot.getBasicTimeStep()) < start_time + duration):
                self.step()

        self.motor_controller.motorStop()
        self.step()


    def moveToDestination(self, destinationCoordinate):
        

        currentCoordinate = self.positioning_controll.positioningControllerGetRobotCoordinate()

        print(currentCoordinate)
        print(destinationCoordinate)
        if cartesianIsCoordinateEqual(currentCoordinate, destinationCoordinate): return 


        thetaDotToDestination = self.positioning_controll.positioningControllerCalcThetaDotToDestination(destinationCoordinate)

        self.rotateHeading(thetaDotToDestination)


        distanceToDestination = self.positioning_controll.positioningControllerCalcDistanceToDestination(destinationCoordinate)

        self.moveForward(distanceToDestination)

        currentCoordinate = self.positioning_controll.positioningControllerGetRobotCoordinate()


"""if __name__ == "__main__":
    init_robot()
    destinationCoordinate = ()

    moveToDestination(destinationCoordinate)"""

    
    