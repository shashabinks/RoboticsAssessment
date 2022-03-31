
from controllers.epuck_controller.cartesian import cartesianIsThetaEqual
from controllers.epuck_controller.motor_controller import motorControllerInit, motorMoveForward, motorRotateLeft, motorRotateRight, motorStop
from epuck_controller import *
from positioning_controller import *
from cartesian import *

TANGENSIAL_SPEED = 0.12874

ROBOT_ROTATIONAL_SPEED = 0.772881647

ROBOT_ANGULAR_SPEED_IN_DEGREES = 278.237392796

time_step = 0.0


def getTimeStep():

    time_step = -1

    if time_step == -1:

        time_step = int(robot.getBasicTimeStep())
    
    return time_step

def step():

    if robot.step(time_step) == -1:
        exit()


def init():

    time_step = getTimeStep()

    motorControllerInit(time_step)

    positioningControllerInit(time_step)


def rotateHeading(thetaDot):

    if(not(cartesianIsThetaEqual(thetaDot,0))):

        duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES

        print(f"duration to face the dest: {duration}")

        if thetaDot > 0: motorRotateLeft()
        elif thetaDot < 0: motorRotateRight()

        start_time = int(robot.getBasicTimeStep())

        while(int(robot.getBasicTimeStep()) < start_time + duration):
            step()


def moveForward(distance):

    duration = distance/TANGENSIAL_SPEED

    motorMoveForward()

    start_time = int(robot.getBasicTimeStep())

    while(int(robot.getBasicTimeStep()) < start_time + duration):
            step()

    motorStop()
    step()


def moveToDestination(destinationCoordinate):
    currentCoordinate = cartesianConvertCompassBearingToHeading(getRobotBearing())

    if cartesianIsCoordinateEqual(cartesianConvertCompassBearingToHeading(), destinationCoordinate): return 


    thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate)

    rotateHeading(thetaDotToDestination)


    distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate)

    moveForward(distanceToDestination)

    currentCoordinate = positioningControllerGetRobotCoordinate()


if __name__ == "__main__":
    init()
    destinationCoordinate = ()

    moveToDestination(destinationCoordinate)

    
    