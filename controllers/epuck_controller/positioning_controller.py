from cartesian import *

import math

GPS_SAMPLE_PERIOD = 1
COMPASS_SAMPLE_PERIOD = 1

global gps, compass


def positioningControllerInit(time_step, robot):
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")

    gps.enable(GPS_SAMPLE_PERIOD)
    compass.enable(COMPASS_SAMPLE_PERIOD)

def getRobotBearing(compass):
    north = compass.getValues()
    rad = math.atan2(north[0],north[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0: bearing = bearing + 360.0

    return bearing

def positioningControllerGetRobotCoordinate():

	return convertVec3ftoVec2f(gps.getValues())


def positioningControllerGetRobotHeading():

	return cartesianConvertCompassBearingToHeading(getRobotBearing(compass))


def positioningControllerCalcDistanceToDestination(destinationCoordinate):

	currentCoordinate = positioningControllerGetRobotCoordinate()
	return cartesianCalcDistance(currentCoordinate, destinationCoordinate)


def positioningControllerCalcThetaDotToDestination(destinationCoordinate):

	currentCoordinate = positioningControllerGetRobotCoordinate()
	robotHeading = positioningControllerGetRobotHeading()
	destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
	return cartesianCalcThetaDot(robotHeading, destinationTheta)