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

colorList = []

count = 0

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
    verticalMotor.run_angle(300, -RAISECLAW)

def openClaw():
    global clawVerticalAngle
    newAngle = clawVerticalAngle - CLAWOPENANGLE
    if (newAngle >= -CLAWMAXVERTICALANGLE):
        clawVerticalAngle = newAngle
        clawMotor.stop()
        clawMotor.run_angle(300, -CLAWOPENANGLE)

def closeClaw():
    global clawVerticalAngle
    clawMotor.run_until_stalled(100,duty_limit=30)
    clawMotor.hold()
    clawVerticalAngle = 0.0
    clawMotor.reset_angle(0.0)
    
def lowerClaw():
    verticalMotor.run_angle(300, RAISECLAW)
    verticalMotor.reset_angle(0.0)


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

def colorZoneSorting(count):
    horizontalMotor.run_angle(200,-360)
    color = findColor()
    
    if color not in colorList and count<3:
        colorList.append(color)
        count += 1
    elif color in colorList:
        index = 0
        for i in range(len(colorList)):
            if color == colorList[i]:
                index = i+1     #+1 ty positionerna räknar vi i heltal och inte från index 0
        position = index*(240)    #60 grader * 4 = 240, plockar upp från höger och 
        horizontalMotor.run_angle(200,position)
    else:   #här får migrationsverket göra sitt arbete
        raiseClaw()
        horizontalMotor.run_angle(700,350, wait=False)
        openClaw()
    horizontalMotor.run_angle(200,360-position)

    

def pickupZone():
    pass



def searchForObject():
    while True:
        openClaw()
        lowerClaw()
        closeClaw()
        raiseClaw()
        colorZoneSorting(count)    #helst ska den återvända till en nollpunkt eller pickup-location innan den börjar sortera




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
    searchForObject()


if __name__ == "__main__":
    main()