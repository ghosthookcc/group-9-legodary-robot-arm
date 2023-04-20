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

robotHorizontalMotorAngle = 0

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

def itemExistsAtLocation(location):
    itemExists = False
    moveToGivenDegree(degreeToMotorAngle)
    #pickup
    if colorSensor.color():
        itemExists = True
        #putdown
    return itemExists

def main() -> int:
    return 0

if __name__ == "__main__":
    main()