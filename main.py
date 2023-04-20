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

def findColor():
    return colorSensor.color()

def main() -> int:
    return 0

def itemExistsAtLocation(location):
    itemExists = False
    #moveToGivenDegree(degreeToMotorAngle)
    #pickup
    if findColor():
        itemExists = True
        #putdown
    return itemExists

if __name__ == "__main__":
    main()