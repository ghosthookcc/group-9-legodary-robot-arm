#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3: EV3Brick = EV3Brick()

clawMotor: Motor = Motor(Port.A)
verticalMotor: Motor = Motor(Port.B)
horizontalMotor: Motor = Motor(Port.C)

colorSensor: ColorSensor = ColorSensor(Port.S2)
touchSensor: TouchSensor = TouchSensor(Port.S1)

verticalMotorAngle = 0.0

clawVerticalAngle = 0.0

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

CLAWMAXHORIZONTALANGLE = 180.0

def calibrate():
    global clawVerticalAngle
    global verticalMotorAngle

    verticalMotor.reset_angle(0)
    raiseClaw()
    closeClaw()
    clawMotor.reset_angle(0)
    clawVerticalAngle = 0.0
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


def exitProgram():
    closeClaw()
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