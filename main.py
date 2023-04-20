#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import os

ev3 = EV3Brick()

clawMotor = Motor(Port.A)
verticalMotor = Motor(Port.B)
horizontalMotor = Motor(Port.C)

colorSensor = ColorSensor(Port.S2)
touchSensor = TouchSensor(Port.S1)

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

HORIZONTALMOTORMAXANGLE = 720.0
HORIZONTALMOTORHALFANGLE = 360.0

CLAWMAXHORIZONTALANGLE = 180.0

class Robot(object):
    def __init__(self):
        self.robotHorizontalMotorAngle = 0.0 
        self.clawVerticalAngle = 0.0
        self.robotVerticalMotorAngle = 0.0
        self.CLAWRESISTANCE = 0.0
        self.ITEMRESISTANCE = 0.0
        self.VERTICALRESISTANCE = 0.0
        self.rotationScale = None

    def findColor(self):
        return colorSensor.color()

    def calibrate(self):
        RobotReset.resetVertical(self)
        RobotReset.resetAll(self)
        
        self.CLAWRESISTANCE = RobotMotors.meassureResistance(self, clawMotor)
        RobotClaw.openClaw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassureResistance(self, verticalMotor)
        RobotClaw.raiseClaw(self)
        RobotClaw.closeClaw(self)

    def userInterface(self):
        while True:
            os.system("clear")
            print("1: Raise Claw")
            print("2: Lower Claw")
            print("3: Open Claw")
            print("4: Close Claw")
            print("5: Find color")
            print("6: Move to degree")
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
                moveToDegree = float(input("Degree: "))
                RobotMotors.moveToGivenDegree(self, moveToDegree)
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
        return angle / self.rotationScale
    def degreeToMotorAngle(self, degree: float) -> float:
        return degree * self.rotationScale
    def moveByGivenMotorAngle(self, horizontalMotorAngle: float, speed: int = 200):
        """
        Raise claw to sensor height and then move by the specified motor angle.
        [Required] motor angle
        [Optional] speed
        """
        RobotClaw.raiseClaw(self)
        self.robotHorizontalMotorAngle += horizontalMotorAngle
        horizontalMotor.run_angle(speed, horizontalMotorAngle)
    def moveByGivenDegree(self, horizontalMotorDegree: float):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then move the horizontal motor by the motor angle.
        [Required] degree
        """
        RobotClaw.raiseClaw(self)
        moveMotorAngle = RobotMotors.degreeToMotorAngle(self, horizontalMotorDegree)
        self.robotHorizontalMotorAngle += moveMotorAngle
        horizontalMotor.run_angle(200, moveMotorAngle)
    def moveToGivenDegree(self, horizontalMotorDegree: float):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then reset the motor to its origin (0 degrees) and rotate to the new degrees,
        degrees is between -90 to 90 degrees from the origin.
        [Required] degree from origin
        """
        RobotClaw.raiseClaw(self)
        horizontalMotor.run_target(150, 0.0)
        moveMotorAngle = RobotMotors.degreeToMotorAngle(self, horizontalMotorDegree)

        RobotMotors.moveByGivenMotorAngle(self, moveMotorAngle)
    
    def meassureResistance(self,motor):
        resistance = motor.run_until_stalled(70,Stop.HOLD,80)
        return resistance
    
class RobotClaw(Robot):
    def raiseClaw(self):
        verticalMotor.run_target(100, -210)

    def lowerClaw(self):
        verticalMotor.run_target(100,self.VERTICALRESISTANCE)

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
        self.openClaw()
        wait(1000)
        self.lowerClaw()
        wait(1000)
        self.closeClaw()
        wait(1000)
        self.raiseClaw()
        wait(1000)

    def dropOffItem(self):
        self.lowerClaw()
        wait(1000)
        self.openClaw()
        wait(1000)
        self.raiseClaw()
        wait(1000)

class RobotReset(Robot):
    def resetHorizontal(self):
        while not touchSensor.pressed():
            RobotMotors.moveByGivenMotorAngle(self, 5.0, 500)
        self.rotationScale = self.robotHorizontalMotorAngle / 90.0
        RobotMotors.moveByGivenMotorAngle(self, -(90.0 * self.rotationScale))
        self.robotHorizontalMotorAngle = 0.0
        horizontalMotor.reset_angle(0.0)

    def resetVertical(self):
        RobotClaw.openClaw(self)
        verticalMotor.run_until_stalled(100, Stop.HOLD, 10)
        self.robotVerticalMotorAngle = 0.0
        verticalMotor.reset_angle(0.0)

    def resetAll(self):
        RobotClaw.closeClaw(self)
        RobotReset.resetHorizontal(self)
        RobotReset.resetVertical(self)

    def exitProgram(self):
        RobotClaw.closeClaw(self)
        RobotReset.resetAll(self)
        os._exit(0)

robot = Robot()

def main():
    robot.manual()

if __name__ == "__main__":
    main()