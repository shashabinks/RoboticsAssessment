
from motor_controller import *
from positioning_controller import *
from cartesian import *

TANGENSIAL_SPEED = 0.12874

ROBOT_ROTATIONAL_SPEED = 0.772881647

ROBOT_ANGULAR_SPEED_IN_DEGREES = 278.237392796

time_step = 0.0


def getTimeStep(robot):

    time_step = -1

    if time_step == -1:

        time_step = int(robot.getBasicTimeStep())
    
    return time_step

def step(robot):

    if robot.step(time_step) == -1:
        exit()


def init_robot(robot):

    time_step = getTimeStep(robot)

    motorControllerInit(time_step, robot)

    positioningControllerInit(time_step, robot)


def rotateHeading(thetaDot, robot):

    if(not(cartesianIsThetaEqual(thetaDot,0))):

        duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES

        print(f"duration to face the dest: {duration}")

        if thetaDot > 0: motorRotateLeft()
        elif thetaDot < 0: motorRotateRight()

        start_time = int(robot.getBasicTimeStep())

        while(int(robot.getBasicTimeStep()) < start_time + duration):
            step(robot)


def moveForward(distance, robot):

    duration = distance/TANGENSIAL_SPEED

    motorMoveForward()

    start_time = int(robot.getBasicTimeStep())

    while(int(robot.getBasicTimeStep()) < start_time + duration):
            step()

    motorStop()
    step()


def moveToDestination(destinationCoordinate, robot):
    

    currentCoordinate = cartesianConvertCompassBearingToHeading(getRobotBearing())

    if cartesianIsCoordinateEqual(cartesianConvertCompassBearingToHeading(), destinationCoordinate): return 


    thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate)

    rotateHeading(thetaDotToDestination, robot)


    distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate)

    moveForward(distanceToDestination, robot)

    currentCoordinate = positioningControllerGetRobotCoordinate()


if __name__ == "__main__":
    init_robot()
    destinationCoordinate = ()

    moveToDestination(destinationCoordinate)

    
    