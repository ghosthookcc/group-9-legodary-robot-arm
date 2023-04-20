#!/usr/bin/env pybricks-micropython
import os
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

clawMotor = Motor(Port.A)
verticalMotor = Motor(Port.B)
horizontalMotor = Motor(Port.C)

colorSensor = ColorSensor(Port.S2)
touchSensor = TouchSensor(Port.S1)

verticalMotorAngle = 0.0

clawVerticalAngle = 0.0

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

CLAWMAXHORIZONTALANGLE = 180.0

RAISECLAW = 195.0




def findColor():
    return colorSensor.color()

def horizontalRotation():
    exitRotation = False
    while exitRotation != True:
        os.system("clear")
        choice = input("Left: l \nRight: r \nExit: 0\n")
        if choice == "r":
            horizontalMotor.run_angle(200,50)
        elif choice == "l":
            horizontalMotor.run_angle(200,-50)
        elif choice == "0":
            exitRotation = True

def decideHorizontalRotation(degree): # Negative number == left, Positive number == right
    horizontalMotor.run_angle(200,degree)   #720 grader är 180 grader, dvs 360 grader är från mitt till en sida




class Robot(object):
    def __init__(self):
        self.robotHorizontalMotorAngle = 0.0 
        self.clawVerticalAngle = 0.0
        self.robotVerticalMotorAngle = 0.0
        self.CLAWRESISTANCE = 0.0
        self.ITEMRESISTANCE = 0.0
        self.VERTICALRESISTANCE = 0.0

    def findColor(self):
        return colorSensor.color()

    def calibrate(self):
        self.CLAWRESISTANCE = RobotMotors.meassureResistance(self, clawMotor)
        RobotClaw.openClaw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassureResistance(self, verticalMotor)
        RobotClaw.raiseClaw(self)
        RobotClaw.closeClaw(self)
        RobotReset.resetHorizontal(self)
        
    def userInterface(self):
        while True:
            os.system("clear")
            print("1: Raise Claw")
            print("2: Lower Claw")
            print("3: Open Claw")
            print("4: Close Claw")
            print("5: FindColor")
            print("6: ColorSorting")
            print("0: Exit")

            answer = input(": ")
            if answer == "1":
                RobotClaw.raiseClaw(self)
            elif answer == "2":
                RobotClaw.lowerClaw(self)
            elif answer =="3":
                RobotClaw.openClaw(self)
            elif answer == "4":
                RobotClaw.closeClaw(self)
            elif answer == "5":
                ev3.light.on(Robot.findColor(self))
            elif answer == "6":
                RobotSorting.colorZoneSorting(self)
            elif answer == "0":
                RobotReset.exitProgram(self)
    
    def manual(self):
        self.calibrate()
        self.userInterface()
    def automate(self):
        self.calibrate()
        while True:
            ev3.light.off()
            RobotClaw.pickupItem(self)
            color = self.findColor(self)
            ev3.light.on(color)
            if (color == Color.RED or 
                color == Color.BLUE or
                color == Color.GREEN or
                color == Color.YELLOW):
                while not (ev3.buttons.pressed()):
                    pass
            RobotClaw.dropOffItem(self)

class RobotMotors(Robot):
    def angleToDegrees(self, angle: float) -> float:
        return angle / 4
    def degreeToMotorAngle(self, degree: float) -> float:
        return degree * 4
    def moveByGivenDegree(self, horizontalMotorAngle: float):
        #motorAngleAsDegree = RobotMotors.angleToDegrees(self, horizontalMotorAngle)
        #horizontalMotor.run_angle(200, motorAngleAsDegree)
        self.robotHorizontalMotorAngle += horizontalMotorAngle
        horizontalMotor.run_angle(200, horizontalMotorAngle)
    
    def meassureResistance(self,motor):
        resistance = motor.run_until_stalled(70,Stop.HOLD,80)
        return resistance
    
class RobotClaw(Robot):
    def raiseClaw(self):
        #verticalMotor.run_angle(200, -195)
        verticalMotor.run_target(100, -210)

    def lowerClaw(self):
        verticalMotor.run_target(100,self.VERTICALRESISTANCE)
        #verticalMotor.reset_angle(0.0)

    def openClaw(self):
        newAngle = self.clawVerticalAngle - CLAWOPENANGLE
        if (newAngle >= -CLAWMAXVERTICALANGLE):
            self.clawVerticalAngle = newAngle
            clawMotor.stop()
            clawMotor.run_angle(300, -CLAWOPENANGLE)

    def closeClaw(self):
        clawMotor.run_until_stalled(100)
        clawMotor.hold()
        self.clawVerticalAngle = 0.0
        clawMotor.reset_angle(0.0)

    def pickupItem(self):
        RobotClaw.openClaw(self)
        wait(1000)
        RobotClaw.lowerClaw(self)
        wait(1000)
        RobotClaw.closeClaw(self)
        wait(1000)
        RobotClaw.raiseClaw(self)
        wait(1000)

    def dropOffItem(self):
        RobotClaw.lowerClaw(self)
        wait(1000)
        RobotClaw.openClaw(self)
        wait(1000)
        RobotClaw.raiseClaw(self)
        wait(1000)

class RobotReset(Robot):


    def resetHorizontal(self):
        while not touchSensor.pressed():
            RobotMotors.moveByGivenDegree(self, 10.0)
        RobotMotors.moveByGivenDegree(self, -self.robotHorizontalMotorAngle)
        self.robotHorizontalMotorAngle = 0.0
        horizontalMotor.reset_angle(0.0)

    def resetVertical(self):
        RobotClaw.openClaw(self)
        verticalMotor.run_until_stalled(100, Stop.HOLD, 10)
        self.robotVerticalMotorAngle = 0.0
        verticalMotor.reset_angle(0.0)

    def resetAll(self):
        RobotReset.resetHorizontal(self)
        RobotReset.resetVertical(self)

    def exitProgram(self):
        RobotClaw.closeClaw(self)
        RobotReset.resetAll(self)
        os._exit(0)

class RobotSorting(Robot):
    colorDict = {}
    positionList = [90,0,-45,-90]   #dessa värden kan ändras till olika positions
    count = 0
    
    def colorZoneSorting(self):
        horizontalMotor.run_target(200,RobotSorting.positionList[0])
        RobotClaw.pickupItem(self)
        color = findColor()
    
        if color not in RobotSorting.colorDict and RobotSorting.count<3:
            RobotSorting.colorDict[color] = RobotSorting.positionList[RobotSorting.count+1]  #+1 för första element är pickupZone
            RobotSorting.count +=1
        for index in range(0,len(RobotSorting.colorDict)):
            pass
        if color in RobotSorting.colorDict.keys:
            position = RobotSorting.colorDict[color]
            horizontalMotor.run_target(200,position)
        
        RobotClaw.dropOffItem(self)

            

    

robot = Robot()

def main():
    robot.manual()

if __name__ == "__main__":
    main()