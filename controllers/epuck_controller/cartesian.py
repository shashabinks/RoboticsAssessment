import math
import cartesian
import positioning_controller

COORDINATE_MATCHING_ACCURACY =  0.01

THETA_MATCHING_ACCURACY = 1


def convertVec3ftoVec2f(coordinate3f):
    coordinate2f = [0.0, 0.0]
    coordinate2f[0] = coordinate3f[0];
    coordinate2f[1] = -coordinate3f[2];
    return coordinate2f;

def convertVec2ftoVec3f(coordinate2f):
    coordinate3f = [0.0, 0.0, 0.0]
    coordinate3f[0] = coordinate2f[0];
    coordinate3f[2] = -coordinate2f[2];
    return coordinate2f;



    
def cartesianConvertCompassBearingToHeading(heading):
    
    heading = 360-heading 

    heading = heading + 90

    if heading > 360.0:
        heading = heading - 360.0 

    return heading

def cartesianIsCoordinateEqual(coordinate1, coordinate2):

    if (abs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY and abs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY):
        return True
    else:
        return False

def cartesianIsCoordinateVectorEqual(coordinate1, coordinate2):

    if (abs(coordinate1-coordinate2) < COORDINATE_MATCHING_ACCURACY):
        return True
    else:
        return False

def cartesianIsThetaEqual(theta1, theta2):

    if (abs(theta1- theta2) < THETA_MATCHING_ACCURACY):
        return True 
    else:
        return False

def cartesianCalcDestinationThetaInDegrees(curr, dest):
    return math.atan2(dest[1]-curr[1], dest[0]-curr[0]) * 180 / 3.14

def cartesianCalcThetaDot(heading,destinationTheta):
    theta_dot = destinationTheta - heading

    if (theta_dot > 180): theta_dot = -(360-theta_dot)
    elif (theta_dot < -180): theta_dot = (360+theta_dot)

    return theta_dot


def cartesianCalcRotatedThetaByThetaDot(theta, theta_dot):

    if theta_dot == 0: return theta

    theta += theta_dot

    if theta < 0: theta = theta + 360
    elif theta >= 360: theta = theta - 360

    return theta

def cartesianCalcDistance(currentCoordinate, destinationCoordinate):
    return math.sqrt((destinationCoordinate[0]-currentCoordinate[0])**2 + (destinationCoordinate[1]-currentCoordinate[1])**2)




