#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

clawMotor = Motor(Port.A)
verticalMotor = Motor(Port.B)
horizontalMotor = Motor(Port.C)

colorSensor = ColorSensor(Port.S2)
touchSensor = TouchSensor(Port.S1)

def calibrate():
    verticalMotor.reset_angle(0)
    verticalMotor.run_angle(300, -215)

def openClaw():
    clawMotor.run_angle(300, -45)

def closeClaw():
    clawMotor.run_until_stalled(300)
    clawMotor.hold()
    
def resetClaw():
    verticalMotor.run_angle(300, 215)
    verticalMotor.reset_angle(0)

def horizontalRotate(speed):
    while (True):
        horizontalMotor.run_angle(speed,-150)
        wait(1000)
        horizontalMotor.run(speed,150)

def userInterface():
    exit = False
    while exit == False:
        print("1: Calibrate")
        print("2: Reset Claw")
        print("3: Open Claw")
        print("4: Close Claw")
        print("5: Exit")

        answer = input(": ")
        if answer == "1":
            calibrate()
        elif answer == "2":
            resetClaw()
        elif answer =="3":
            openClaw()
        elif answer == "4":
            closeClaw()
        elif answer == "5":
            exit = True

def main() -> int:
    userInterface()
    return 0

if __name__ == "__main__":
    main()