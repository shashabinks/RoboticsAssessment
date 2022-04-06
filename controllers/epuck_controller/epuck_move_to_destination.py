

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
        self.current_rotation = 0

        self.correct_rotation = None
        

    def get_move_dir(self, correct_rotation, current_rotation):
        
        move_dir = 0

        behind_me = current_rotation - (math.pi/2)

        if behind_me < 0:
            behind_me = behind_me + math.pi

        if abs(correct_rotation - behind_me) < 0.05:
            move_dir = 1

        elif (correct_rotation > behind_me and correct_rotation < current_rotation) or (current_rotation < math.pi/2 and (correct_rotation > behind_me or correct_rotation < current_rotation)):
            move_dir = -1
        
        elif (correct_rotation < behind_me and correct_rotation > current_rotation) or (current_rotation > math.pi/2 and (correct_rotation < behind_me or correct_rotation > current_rotation)):
            move_dir = 1
        
        return move_dir

    def rotateHeading(self):
        
        # this is from the properties of the robot itself
        #current_rotation = self.robot.getSelf().getField("rotation").getSFRotation()[3]

        #wrap_rotation = (current_rotation + math.pi) % (2 * math.pi) - math.pi
        #wrap_correct_rotation = (self.correct_rotation + math.pi) % (2 * math.pi) - math.pi
        #45 = 0.785
        #each timestep = 0.0568
        print("curr: " + str(self.current_rotation) + " target: " + str(self.correct_rotation))
        if abs(self.current_rotation - self.correct_rotation) > 0.02259:

            #TODO radian signs are not wrapping around correctly, find a way to track the sign ourselves and update the current rotation sign accordingly
            #graph of what it should be https://uk.mathworks.com/help/map/ref/wraptopi.html
            #example of bug: curr is current rotation, target is correct_rotation, rotation direction is left in this case
            #curr: -0.3457148179076146 target: 2.356386591841484
            #curr: -0.28947520829371687 target: 2.356386591841484
            #curr: -0.23324916427643316 target: 2.356386591841484
            #curr: -0.17703626284140572 target: 2.356386591841484
            #curr: -0.12083618676476426 target: 2.356386591841484
            #curr: -0.06464871323623367 target: 2.356386591841484
            #curr: -0.008473705794251885 target: 2.356386591841484
            #curr: -0.04768870377706946 target: 2.356386591841484
            #curr: -0.10383955011457946 target: 2.356386591841484
            #curr: -0.16219249933397784 target: 2.356386591841484

            #chosen direction fixed
            if self.rotation_dir is None:
                if self.correct_rotation > 0:
                    self.rotation_dir = 1
                else: self.rotation_dir = -1

                

            if self.rotation_dir == 1:
                self.motor_controller.motorRotateLeft()
                self.current_rotation+= 0.02259

            else:
                self.motor_controller.motorRotateRight()
                self.current_rotation-= 0.02259
        
        else:
            self.motor_controller.motorStop()
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

        
        if cartesianIsCoordinateEqual(currentCoordinate, destinationCoordinate) or self.forward_done:
            self.destination_set = False

            self.rotation_done = False
            self.forward_done = False

            self.rotation_duration = None
            self.forward_duration = None

            self.rotation_start = None
            self.forward_start = None
            self.rotation_dir = None

            self.correct_rotation = None
            self.current_rotation = 0
            self.motor_controller.motorStop()
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

    
    