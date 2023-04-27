#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import os
import time

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
        self.VERTICALRESISTANCE = 0.0
        self.rotationScale = None
        RobotCalibrate.calibrate(self)

    def findColor(self):
        return colorSensor.color()

    def calibrate(self):
        RobotReset.resetVertical(self)
        RobotReset.resetAll(self)

        clawMotor.run_until_stalled(100)
        clawMotor.reset_angle(0.0)
        self.clawVerticalAngle = 0.0

        RobotClaw.openClaw(self)
        self.CLAWRESISTANCE = RobotMotors.meassureResistance(self, clawMotor)
        RobotClaw.openClaw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassureResistance(self, verticalMotor)

        RobotClaw.raiseClaw(self)
        RobotClaw.closeClaw(self)

    def userInterface(self):
        while True:
            #input(": Press any key :")
            os.system("clear")
            print("1: Raise Claw")
            print("2: Lower Claw")
            print("3: Open Claw")
            print("4: Close Claw")
            print("5: Find color")
            print("6: Move to degree")
            print("7: Check if a item is being held")
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
                motor = int(input("Motor (1 horizontal, 2 vertical): "))
                moveToDegree = float(input("Degree: "))
                if (motor == 1):
                    RobotMotors.moveToGivenDegree(self, horizontalMotor, moveToDegree)
                elif (motor == 2):
                    RobotMotors.moveToGivenDegree(self, verticalMotor, moveToDegree)
            elif answer == "7":
                isHolding = RobotClaw.isHoldingItem(self)
                print("Item is being held: " + str(isHolding))
            elif answer == "0":
                RobotCalibrate.exitProgram(self)
    
    def manual(self):
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
    def moveByGivenMotorAngle(self, motor, motorAngle: float, speed: int = 200):
        """
        Raise claw to sensor height and then move by the specified motor angle.
        [Required] motor
        [Required] motor angle
        [Optional] speed
        """

        if (motor == horizontalMotor):
            RobotClaw.raiseClaw(self)
            self.robotHorizontalMotorAngle += motorAngle
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle += motorAngle
        motor.run_angle(speed, motorAngle)
    def moveByGivenDegree(self, motor, motorDegree: float):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then move the horizontal motor by the motor angle.
        [Required] motor
        [Required] degree
        """
        motorAngle = RobotMotors.degreeToMotorAngle(self, motorDegree)

        if (motor == horizontalMotor):
            RobotClaw.raiseClaw(self)
            self.robotHorizontalMotorAngle += motorAngle
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle += motorAngle
        horizontalMotor.run_angle(200, motorAngle)
    def moveToGivenDegree(self, motor, motorDegree: float):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then reset the motor to its origin (0 degrees) and rotate to the new degrees,
        degrees is between -90 to 90 degrees from the origin.
        [Required] motor
        [Required] degree from origin
        """
        motor.run_target(150, 0.0)
        moveMotorAngle = RobotMotors.degreeToMotorAngle(self, motorDegree)

        RobotMotors.moveByGivenMotorAngle(self, motor, moveMotorAngle)
    
    def meassureResistance(self, motor):
        """
        Meassure resistance of a motor by running the motor until stalled at a angle of atleast 80%
        """
        resistance = motor.run_until_stalled(70,Stop.HOLD,80)
        return resistance
    
class RobotClaw(Robot):
    def raiseClaw(self):
        """
        Raise claw on the vertical axis to motor angle of the color sensor
        """
        verticalMotor.run_target(100, -210.0)

    def lowerClaw(self):
        """
        Lower claw on vertical axis to default position after calibration
        """
        verticalMotor.run_target(100, 0.0)

    def openClaw(self):
        """
        Open claw if the value is not bigger than the max claw motor angle
        """
        newAngle = self.clawVerticalAngle - CLAWOPENANGLE
        if (newAngle >= -CLAWMAXVERTICALANGLE):
            self.clawVerticalAngle = newAngle
            clawMotor.stop()
            clawMotor.run_target(300, -CLAWOPENANGLE)

    def closeClaw(self):
        """
        Close claw by running the claw motor until stalled and then resetting the angle to 0
        """
        clawMotor.run_target(50, 0.0)
        clawMotor.hold()
        self.clawVerticalAngle = 0.0
        clawMotor.reset_angle(0.0)

    def isHoldingItem(self) -> bool:
        isHolding = False
        t_end = time.time() + 3.0
        stepping = self.clawVerticalAngle / 5.0
        while (-90.0 <= self.clawVerticalAngle <= 90.0):
            print(self.clawVerticalAngle)
            if (time.time() > t_end):
                isHolding = True
                break
            self.clawVerticalAngle += stepping
            clawMotor.run_target(100, self.clawVerticalAngle)
        return isHolding

        #isHolding = False
        #resistance = RobotMotors.meassureResistance(self, clawMotor)
        #print(str(resistance) + " : " + str(self.ITEMRESISTANCE) + " : " + str(self.CLAWRESISTANCE))
        #if self.ITEMRESISTANCE-8 <= resistance <= self.ITEMRESISTANCE+8:
        #    isHolding = True
        #return isHolding

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

class RobotCalibrate(Robot):
    def calibrate(self):
        RobotCalibrate.resetVertical(self)
        RobotCalibrate.resetAll(self)
        
        self.CLAWRESISTANCE = RobotMotors.meassureResistance(self, clawMotor)
        RobotClaw.openClaw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassureResistance(self, verticalMotor)
        RobotClaw.raiseClaw(self)
        RobotClaw.closeClaw(self)
    
    def calinrateZones():
        zonelst = ["Pickup Zone", "Drop off Zone 1","Drop off Zone 2", "Drop off Zone 3" ]
        for index, i in enumerate(zonelst):
            ev3.screen.draw_text(40, 50,i)
            while not ev3.buttons.pressed()[0] == Button.CENTER:
                pressedButton = ev3.buttons.pressed()[0]
                if pressedButton == Button.UP:
                    verticalMotor.run_angle(100, 10)
                elif pressedButton == Button.DOWN:
                    verticalMotor.run_angle(100,-10)
                elif pressedButton == Button.RIGHT:
                    horizontalMotor.run_angle(100,10)
                elif pressedButton == Button.LEFT:
                    horizontalMotor.run_angle(100,-10)
            zonelst[index] = ()

        

    def resetHorizontal(self):
        while not touchSensor.pressed():
            RobotMotors.moveByGivenMotorAngle(self, horizontalMotor, 5.0, 500)
        self.rotationScale = self.robotHorizontalMotorAngle / 90.0
        RobotMotors.moveByGivenMotorAngle(self, horizontalMotor, -(90.0 * self.rotationScale))
        self.robotHorizontalMotorAngle = 0.0
        horizontalMotor.reset_angle(0.0)

    def resetVertical(self):
        verticalMotor.run_until_stalled(100, Stop.HOLD, 10)
        self.robotVerticalMotorAngle = 0.0
        verticalMotor.reset_angle(0.0)

    def resetAll(self):
        RobotClaw.closeClaw(self)
        RobotCalibrate.resetHorizontal(self)
        RobotCalibrate.resetVertical(self)

    def exitProgram(self):
        RobotClaw.closeClaw(self)
        RobotCalibrate.resetAll(self)
        os._exit(0)

robot = Robot()

def main():
    robot.manual()

if __name__ == "__main__":
    main()