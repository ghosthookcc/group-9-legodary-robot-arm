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
    clawMotor.run_angle(30, -45)
def closeClaw():
    clawMotor.run_until_stalled(30)
    
def resetClaw():
    verticalMotor.run_angle(300, 215)
    verticalMotor.reset_angle(0)

def horizontalRotate(speed):
    while (True):
        horizontalMotor.run_angle(speed,-150)
        wait(1000)
        horizontalMotor.run(speed,150)


def main() -> int:
    calibrate()
    openClaw()
    wait(1000)
    closeClaw()
    return 0

if __name__ == "__main__":
    main()