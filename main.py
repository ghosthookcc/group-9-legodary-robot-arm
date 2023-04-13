#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import os

clawMotor = Motor(Port.A)
verticalMotor = Motor(Port.B)
horizontalMotor = Motor(Port.C)

colorSensor = ColorSensor(Port.S2)
touchSensor = TouchSensor(Port.S1)

horizontalMotorAngle = 0.0
verticalMotorAngle = 0.0

clawVerticalAngle = 0.0

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

HORIZONTALMOTORMAXANGLE = 720.0
HORIZONTALMOTORHALFANGLE = 360.0

def calibrate():
    global clawVerticalAngle
    global verticalMotorAngle
    global horizontalMotorAngle

    verticalMotor.reset_angle(0)
    raiseClaw()
    closeClaw()
    clawMotor.reset_angle(0)
    clawVerticalAngle = 0.0
    resetHorizontal()

def resetHorizontal():
    global horizontalMotorAngle
    while not touchSensor.pressed():
        horizontalMotorAngle += 10.0
        horizontalRotation(10.0) 
    horizontalRotation(-horizontalMotorAngle / 2.0)
    horizontalMotorAngle = 0.0
    horizontalMotor.reset_angle(0)

def raiseClaw():
    verticalMotor.run_angle(300, -215)

def openClaw():
    global clawVerticalAngle
    newAngle = clawVerticalAngle - CLAWOPENANGLE
    if (newAngle >= -CLAWMAXVERTICALANGLE):
        clawVerticalAngle = newAngle
        clawMotor.stop()
        clawMotor.run_angle(300, -CLAWOPENANGLE)

def closeClaw():
    global clawVerticalAngle
    clawMotor.run_until_stalled(300)
    clawMotor.hold()
    clawVerticalAngle = 0.0
    clawMotor.reset_angle(0.0)
    
def lowerClaw():
    verticalMotor.run_angle(300, 215)
    verticalMotor.reset_angle(0.0)

def horizontalRotation(degree): # Negative number == left, Positive number == right
    global horizontalMotorAngle
    horizontalMotorAngle += degree
    horizontalMotor.run_angle(200,degree)   #720 grader är 180 grader, dvs 360 grader är från mitt till en sida

def exitProgram():
    closeClaw()
    resetHorizontal()
    verticalMotor.run_until_stalled(200)
    os._exit(0)

def userInterface():
    while True:
        os.system("clear")
        print("1: Raise Claw")
        print("2: Lower Claw")
        print("3: Open Claw")
        print("4: Close Claw")
        print("5: Rotate Claw")
        print("6: Position Claw (Horizontal)")
        print("0: Exit")

        answer = input(": ")
        if answer == "1":
            raiseClaw()
        elif answer == "2":
            lowerClaw()
        elif answer =="3":
            openClaw()
        elif answer == "4":
            closeClaw()
        elif answer == "5":
            horizontalRotation()
        elif answer == "6":
            degree = int(input("Give degree: "))    #här behöver vi fixa om de skriver en string, catch
            horizontalRotation(degree)
        elif answer == "0":
            exitProgram()

def main():
    calibrate()
    userInterface()


if __name__ == "__main__":
    main()