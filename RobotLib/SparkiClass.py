import numpy as np
import math
import pygame
import time
from .Math import *
from .IO import *

radiusOfWheelAxis = 8.41 / 2    #cm
sizeOfOneStep = .0038           #cm, can change this if the radius is different
                                #2 * pi * radius / 4096

class SparkiClass(object):

    def __init__(self,width=256,height=256,theta=0):
        # _indicates should not change straight up, use functions
        # no _ is ok to change on their own
        self.velocity = 0.
        self.omega = 0.
        self.theta = 0.

        self._sonarReadingS = vec(25.,0.)

        self._leftVelocity = 0.
        self._rightVelocity = 0.
        self.leftWheelDir = 0
        self.rightWheelDir = 0

        self._backRightR = vec(-5.,-4.5)
        self._backLeftR = vec(-5.,4.5)
        self._frontRightR = vec(5.,-4.5)
        self._frontLeftR = vec(5,4.5)
        self._headLocationR = vec(2.5,0.)
        self._centerR = vec(0.,0.)
    
        self._centerM = vec(width/2.,height/2.)

        self._transRtoM = transform(width/2.,height/2.,self.theta)
        self._transStoR = transform(2.5,0.,0.)
        self._transMtoR = invert(self._transRtoM) 
        self._transRtoS = invert(self._transStoR)
        
        self.hasRun = False
    
    #getters for points in map frame
    def backRightM(self):
        return mul(self._transRtoM,self._backRightR)
        
    def backLeftM(self):
        return mul(self._transRtoM,self._backLeftR)

    def frontRightM(self):
        return mul(self._transRtoM,self._frontRightR)

    def frontLeftM(self):
        return mul(self._transRtoM,self._frontLeftR)

    def centerM(self):
        return mul(self._transRtoM,self._centerR)

    def headLocationM(self):
        return mul(self._transRtoM,self._headLocationR)

    def sonarReadingM(self):
        return mul(self._transRtoM,mul(self._transStoR,self._sonarReadingS))

    def sonarDistance(self,distance):
        self._sonarReadingS[0] = distance


    # circumference of wheel = 15.71 cm
    # 4096 steps per revolution.
    # 1 step =.0038 cm /step
    # max speed is 1000 steps per sec or 3.8 cm per sec
    # 90% is 900 or 3.42 cm per sec
    def getCommandLeft(self):
        if self._leftVelocity > (sizeOfOneStep * 100):
            return 100
        else:
            return int(self._leftVelocity / sizeOfOneStep)

    def getCommandRight(self):
        if self._rightVelocity > (sizeOfOneStep * 100):
            return 100
        else: 
            return int(self._rightVelocity / sizeOfOneStep)


    #update functions
    def updateTransforms(self):
        self._transRtoM = transform(self._centerM[0],self._centerM[1],self.theta) np.matrix[self.ceterM[0]-ICCx,self.centerM[1]-ICCy,theta] + np.matrix[Iccx,Iccy,theta]
        self._transMtoR = invert(self._transRtoM)

    def updateRightVelocity(self):
        self._rightVelocity = self.velocity + (self.omega * (8.52/2) )
        if self._rightVelocity < 0 :
            self._rightVelocity = abs(self._rightVelocity)
            self.rightWheelDir = 1

    def updateLeftVelocity(self):
        self._leftVelocity = self.velocity - (self.omega * (8.52/2) )
        if self._leftVelocity < 0 :
            self._leftVelocity = abs(self._leftVelocity)
            self.leftWheelDir = 1

    def updateCenter(self,time_delta):
        self.theta += self.omega * time_delta
        self._centerM[0] += self.velocity * math.cos(self.theta) * time_delta
        self._centerM[1] += self.velocity * math.sin(self.theta) * time_delta
        #create a move matrix, then multiply by center?
        #what is the radius? how does that matter?
        self.updateRightVelocity()
        self.updateLeftVelocity()
        self.updateTransforms()
        self.ICCx
        self.ICCy
        if velcoityL == - VelocityR
             ICCr = 0
        
        if VelL ==0 or VelR == 0
            R = radiusOfWheelAxis

        if VelL == VelR
            #r goes to infinity
            ICCx = _centerM[0]
            ICCy = _centerM[1]
        else
            ICCx = _centerM[0] - ICCr * math.sin(theta)
            ICCy = _centerM[1[ + ICCr * math.cos(theta)
        
        
    def testPrint(self):
        print ("it worked") 

    #call draw to draw your robot including sonar
    def draw(self,surface):
        pygame.draw.line(surface,(0,0,255),self.frontRightM(),self.frontLeftM())
        pygame.draw.line(surface,(0,255,0),self.frontRightM(),self.backRightM())
        pygame.draw.line(surface,(0,255,0),self.frontLeftM(),self.backLeftM())
        pygame.draw.line(surface,(0,255,0),self.backLeftM(),self.backRightM())
        pygame.draw.line(surface,(255,0,0),self.headLocationM(),self.sonarReadingM())
        

    #functions to move sparky a set distance or rotate by a set angle.
    #will cause unknown behavior in the simulation
    def move(self,sparki,distance,power):
        leftDir = 0
        rightDir = 0
        #don't overload power and check for reverse
        if power > 100:
            power = 100
        elif power < 0:
            power = abs(power)
            leftDir = 1
            rightDir = 1

        if power > 100:
            power = 100
        
        moveTime = distance / (sizeOfOneStep * power)
        print("We will move for %5.3f sec" % moveTime)
        sparki.send_command(power,leftDir,power,rightDir,0,0) #send move command
        time.sleep(moveTime) #time in seconds, goes to millisec accuracy
        sparki.send_command(0,0,0,0,0,0) #send stop command
        print("We moved %.2f cm in %5.3f sec at %d power" %(distance,moveTime,power))

    def rotate(self,sparki,theta,power):
        #if positive then set right wheel to have power
        #if neg then set left wheel power
        #possibley add reverse power to other wheels to spin on center point, start with one wheel
        leftDir = 0
        rightDir = 0

        #don't overload power
        if power > 100:
            power = 100
        elif power < 0:
            power = abs(power)

        if power > 100:
            power = 100
        
        #set direction of rotation by powering opposite wheel
        if theta > 0:
            leftPower = 0
            rightPower = power
        else:
            leftPower = power
            rightPower = 0   

        #will not rotate more than 360 degrees
        if theta > 360 or theta < -360:
            theta = theta % 360
             
        #distance the wheel should move to get desired angle
        arcLength = 2. * math.pi * radiusOfWheelAxis * (abs(theta)/360)
        
        moveTime =  arcLength / (sizeOfOneStep * power)
        print("We will move for %5.3f sec" % moveTime)
        sparki.send_command(int(leftPower),leftDir,int(rightPower),rightDir,0,0) #send move command
        time.sleep(moveTime) #time in seconds, goes to millisec accuracy
        sparki.send_command(0,0,0,0,0,0) #send stop command
        print("We rotated %.2f degrees in %5.3f sec at %d power" %(theta,moveTime,power))

