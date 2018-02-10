import pygame
import sys
import time
from RobotLib.Math import *
import math
import argparse
from RobotLib.FrontEnd import *
from RobotLib.IO import *
import numpy as np
from SparkiClass import *

#if using only simulation, allows sparki to move faster
simulationONLY = False

#if using the move commands to move sparki a set distance
movingSparkiSetDistance = False

class MyFrontEnd(FrontEnd):
    """ Custom sub-class of FrontEnd
        You can write custom sub-routines here to respond to UI events and calculate updates
    """

    #custom sparki class
    MySparkiClass = SparkiClass()

    def __init__(self,width,height,sparki):
        FrontEnd.__init__(self,width,height)
        self.sparki = sparki 
                    
    def mouseup(self,x,y,button):
        # x,y is position of mouse click
        print('mouse clicked at %d, %d'%(x,y))

    def keydown(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP 
        #set velocities based on pressing of keys. 90% forward and back. small angular velocity to test
        if key == 273: 
            if simulationONLY:
                self.MySparkiClass.velocity += 15
            else:
                self.MySparkiClass.velocity += 3.04
        if key == 274:
            if simulationONLY:
                self.MySparkiClass.velocity -= 15
            else:
                self.MySparkiClass.velocity -= 3.04
        if key == 275:
            if simulationONLY:
                self.MySparkiClass.omega -= 1
            else:
                self.MySparkiClass.omega -= .8
        if key == 276:
            if simulationONLY:
                self.MySparkiClass.omega += 1
            else:
                self.MySparkiClass.omega += .8


    def keyup(self,key):
        # see https://www.pygame.org/docs/ref/key.html for pygame key names, such as pygame.K_UP
        #set velocities to 0 on release of keys
        if key == 273:
            self.MySparkiClass.velocity = 0
        if key == 274:
            self.MySparkiClass.velocity = 0
        if key == 275:
            self.MySparkiClass.omega = 0
        if key == 276: 
            self.MySparkiClass.omega = 0
 
        
    def draw(self,surface):
        # draw robot here
        #
        # draw a rectangle for the robot
        # draw a red line from the sonar to the object it is pinging
        #
        # use pygame.draw.line(surface,color,point1,point2) to draw a line
        # for example, pygame.draw.line(surface,(0,0,0),(0,0),(10,10))
        # draws a black line from (0,0) to (10,0)
 
       
        #sends draw command in custom class for simulation
        self.MySparkiClass.draw(surface)      

    def update(self,time_delta):
        # this function is called approximately every 50 milliseconds
        # time_delta is the time in seconds since the last update() call
        # 
        # you can send an update command to sparki here
        # use self.sparki.send_command(left_speed,left_dir,right_speed,right_dir,servo,gripper_status)
        # see docs in RobotLib/IO.py for more information, #0 for forward. #0 for strop gripper_status
        #
        # if you send a message more than once per millisecond, the message will be dropped
        # so, only call send_command() once per update()
        #
        # you can also calculate dead reckoning (forward kinematics) and other things like PID control here


        # if connected to sparki, get sonar reading and send commands to sparki
        if simulationONLY != True:
            self.MySparkiClass.sonarDistance(self.sparki.dist) #will show a point if reading 0
            self.sparki.send_command(self.MySparkiClass.getCommandLeft(),self.MySparkiClass.leftWheelDir,
                self.MySparkiClass.getCommandRight(),self.MySparkiClass.rightWheelDir,0,0)
             
        # updates sparki's location for simulation
        self.MySparkiClass.updateCenter(time_delta)
        
        # move sparki a set distance, can put multiple commands to run in sequence
        # does not translate this movement to the simulation, used for part 4 of HW 1
        # can possibly be expanded later
        if movingSparkiSetDistance == True and self.MySparkiClass.hasRun == False:
            # (sparki, dist in cm, power)
            self.MySparkiClass.move(self.sparki,10,90)
            # (sparki, theta in degrees, power)
            self.MySparkiClass.rotate(self.sparki,90,90)
            self.MySparkiClass.hasRun = True #do not change this or you will be locked out from system pause

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='Template')
    parser.add_argument('--width', type=int, default=256, help='map width')
    parser.add_argument('--height', type=int, default=256, help='map height')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()
    
    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(args.width,args.height,sparki)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
