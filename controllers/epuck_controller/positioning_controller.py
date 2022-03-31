from msilib.schema import Class
from cartesian import *

import math

GPS_SAMPLE_PERIOD = 1
COMPASS_SAMPLE_PERIOD = 1


class Positioning_controller:

    def __init__(self, robot):
        self.robot = robot
        self.gps = robot.getDevice("gps")
        self.compass = robot.getDevice("compass")

        self.gps.enable(GPS_SAMPLE_PERIOD)
        self.compass.enable(COMPASS_SAMPLE_PERIOD)
    

        

    def getRobotBearing(self):
        north = self.compass.getValues()
        rad = math.atan2(north[0],north[2])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0.0: bearing = bearing + 360.0

        return bearing

    def positioningControllerGetRobotCoordinate(self):

        return convertVec3ftoVec2f(self.gps.getValues())


    def positioningControllerGetRobotHeading(self):

        return cartesianConvertCompassBearingToHeading(self.getRobotBearing())


    def positioningControllerCalcDistanceToDestination(self, destinationCoordinate):

        currentCoordinate = self.positioningControllerGetRobotCoordinate()
        return cartesianCalcDistance(currentCoordinate, destinationCoordinate)


    def positioningControllerCalcThetaDotToDestination(self, destinationCoordinate):

        currentCoordinate = self.positioningControllerGetRobotCoordinate()
        robotHeading = self.positioningControllerGetRobotHeading()
        
        
        destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
       
        return cartesianCalcThetaDot(robotHeading, destinationTheta)