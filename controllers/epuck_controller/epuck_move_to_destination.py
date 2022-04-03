

from numpy import arctan2
from motor_controller import *
from positioning_controller import *
from cartesian import *




class Epuck_move:
    def __init__(self, robot, rot_speed, forward_speed):
        self.robot = robot
        self.rot_speed = rot_speed
        self.forward_speed = forward_speed
        self.motor_controller = Motor_Controller(self.robot, rot_speed, forward_speed)
        self.positioning_controll = Positioning_controller(self.robot)
        self.destination_set = False

        self.rotation_done = False
        self.forward_done = False

        self.rotation_duration = None
        self.forward_duration = None

        self.rotation_start = None
        self.forward_start = None
        self.rotation_dir = None

        self.correct_rotation = None
        

    def get_move_dir(self, correct_rotation, current_rotation):
        
        move_dir = 0

        behind_me = current_rotation - (math.pi/2)

        if behind_me < 0:
            behind_me = behind_me + math.pi;

        if abs(correct_rotation - behind_me) < 0.05:
            move_dir = 1

        elif (correct_rotation > behind_me and correct_rotation < current_rotation) or (current_rotation < math.pi/2 and (correct_rotation > behind_me or correct_rotation < current_rotation)):
            move_dir = -1
        
        elif (correct_rotation < behind_me and correct_rotation > current_rotation) or (current_rotation > math.pi/2 and (correct_rotation < behind_me or correct_rotation > current_rotation)):
            move_dir = 1
        
        return move_dir

    def rotateHeading(self):
        
        current_rotation = self.robot.getSelf().getField("rotation").getSFRotation()[3]
        print("curr: " + str(current_rotation) + " target: " + str(self.correct_rotation))
        if abs(current_rotation - self.correct_rotation) > 0.05:

            #fixed
            if self.rotation_dir is None:
                self.rotation_dir = self.get_move_dir(self.correct_rotation, current_rotation)

            if self.rotation_dir is 1:
                self.motor_controller.motorRotateLeft()

            else:
                self.motor_controller.motorRotateRight()
        
        else:
            self.rotation_done = True
            

    def moveForward(self, distance):

        if self.forward_duration == None:
            tangelnsial_speed = 0.0205 * self.forward_speed
            self.forward_duration = distance/tangelnsial_speed
            self.forward_start = self.robot.getTime()

        if self.robot.getTime() > self.forward_start + self.forward_duration:
            self.motor_controller.motorStop()
            self.forward_done = True
            return

        print("currentTime: " + str(self.robot.getTime()), " start + duration: " + str(self.forward_start + self.forward_duration) + " forward: " + str(self.forward_duration))
        self.motor_controller.motorMoveForward()

    def calculate_rotation(self, current_cords, destination_cords):

        angle = arctan2(destination_cords[1] - current_cords[1], destination_cords[0] - current_cords[0])

        return angle

    def moveToDestination(self, destinationCoordinate):
        
        if self.destination_set == False:
            epuck_ref = self.robot.getSelf()
            self.destination_set = True
        
            self.correct_rotation = self.calculate_rotation(epuck_ref.getField("translation").getSFVec3f()[0:2], destinationCoordinate)
            

            
            
            
            #https://stackoverflow.com/questions/7846775/how-to-gradually-rotate-an-object-to-face-another-turning-the-shortest-distance
            
            
            
            
            

        currentCoordinate = self.positioning_controll.positioningControllerGetRobotCoordinate()

        
        if cartesianIsCoordinateEqual(currentCoordinate, destinationCoordinate):
            self.destination_set = False

            self.rotation_done = False
            self.forward_done = False

            self.rotation_duration = None
            self.forward_duration = None

            self.rotation_start = None
            self.forward_start = None
            self.rotation_dir = None

            self.correct_rotation = None
            return True

        

        if not self.rotation_done:
            
            self.rotateHeading()


    

        if not self.forward_done and self.rotation_done:
            distanceToDestination = self.positioning_controll.positioningControllerCalcDistanceToDestination(destinationCoordinate)
            self.moveForward(distanceToDestination)


        return False


"""if __name__ == "__main__":
    init_robot()
    destinationCoordinate = ()

    moveToDestination(destinationCoordinate)"""

    
    