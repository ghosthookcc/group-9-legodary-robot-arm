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

robotHorizontalMotorAngle = 0.0
verticalMotorAngle = 0.0

clawVerticalAngle = 0.0

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

HORIZONTALMOTORMAXANGLE = 720.0
HORIZONTALMOTORHALFANGLE = 360.0

CLAWMAXHORIZONTALANGLE = 180.0

def findColor():
    return colorSensor.color()

def motorAngleToDegrees(angle: float) -> float:
    return angle / 4

def degreeToMotorAngle(degree: float) -> float:
    return degree * 4

def moveToGivenDegree(horizontalPositionDegree: any):
    global robotHorizontalMotorAngle
    itemHorizontalMotorAngle = degreeToMotorAngle(horizontalPositionDegree)
    if itemHorizontalMotorAngle < robotHorizontalMotorAngle:
        moveAngle = abs(robotHorizontalMotorAngle) + abs(itemHorizontalMotorAngle)
        horizontalMotor.run_angle(200, (-moveAngle))
    elif itemHorizontalMotorAngle > robotHorizontalMotorAngle:
        moveAngle = abs(robotHorizontalMotorAngle) + abs(itemHorizontalMotorAngle)
        horizontalMotor.run_angle(200, (moveAngle))

def calibrate():
    global clawVerticalAngle
    global verticalMotorAngle
    global robotHorizontalMotorAngle

    verticalMotor.reset_angle(0.0)

    raiseClaw()
    closeClaw()

    resetHorizontal()

def resetHorizontal():
    global robotHorizontalMotorAngle
    while not touchSensor.pressed():
        robotHorizontalMotorAngle += 10.0
        horizontalRotation(10.0) 
    horizontalRotation(-robotHorizontalMotorAngle / 2.0)
    robotHorizontalMotorAngle = 0.0
    horizontalMotor.reset_angle(0)

def raiseClaw():
    verticalMotor.run_angle(200, -195)

def lowerClaw():
    verticalMotor.run_angle(200, 195)
    verticalMotor.reset_angle(0.0)

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

def horizontalRotation(degree): # Negative number == left, Positive number == right
    global robotHorizontalMotorAngle
    robotHorizontalMotorAngle += degree
    horizontalMotor.run_angle(200, degree)

def exitProgram():
    closeClaw()
    resetHorizontal()
    verticalMotor.run_until_stalled(200)
    os._exit(0)

def pickupItem():
    openClaw()
    wait(3000)
    lowerClaw()
    wait(3000)
    closeClaw()
    wait(3000)
    raiseClaw()
    wait(3000)

def dropOffItem():
    lowerClaw()
    wait(3000)
    openClaw()
    wait(3000)
    raiseClaw()
    wait(3000)

def robotAutomation():
    while True:
        ev3.light.off()
        pickupItem()
        color = findColor()
        ev3.light.on(color)
        print(color)
        if ((color == Color.GREEN) 
            or (color == Color.RED) 
            or (color ==  Color.BLUE)):
            while not (ev3.buttons.pressed()):
                pass
        dropOffItem()

def demofunction(): #Temp function to show off use cases for sprint 1
    while True:
        if ev3.buttons.pressed(4):
            pickupItem()
        elif ev3.buttons.pressed(5):
            dropOffItem()
        elif ev3.buttons.pressed(2):
            ev3.light.on(findColor())

def userInterface():
    while True:
        os.system("clear")
        print("1: Raise Claw")
        print("2: Lower Claw")
        print("3: Open Claw")
        print("4: Close Claw")
        print("5: Pick up Item")
        print("6: Drop off Item")
        print("7: Prototype Automation")
        print("8: Button Control")
        print("9: Color")
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
            pickupItem()
        elif answer == "6":
            dropOffItem()
        elif answer == "7":
            robotAutomation()
        elif answer == "8":
            demofunction()
        elif answer == "9":
            ev3.light.on(findColor())
        elif answer == "0":
            exitProgram()

def main():
    calibrate()
    robotAutomation()


if __name__ == "__main__":
    main()