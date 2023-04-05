#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

clawMotor: Motor = Motor(Port.A)
verticalMotor: Motor = Motor(Port.B)
horizontalMotor: Motor = Motor(Port.C)

colorSensor: ColorSensor = ColorSensor(Port.S2)
touchSensor: TouchSensor = TouchSensor(Port.S1)

def resetClaw():
    verticalMotor.run_target(300,-150)
    #clawMotor.run_angle(500,20)
    pass

def interactWithItem():

    clawMotor.run_angle(500,-50)
    wait(1000)
    verticalMotor.run_target(300,250)
    wait(1000)
    #clawMotor.run_target(500,50, wait=False)    #denna måste vara false när den plockar upp 
    clawMotor.run_until_stalled

    verticalMotor.run_target(300,-250)
    
    
    #wait(1000)
    
    #while True:
        #clawMotor.run_angle(500,i*10)   #första värde är grader/sekund, andra är hur många grader som öppnas
        #wait(10)

def main() -> int:
    #dropItem()
    resetClaw()
    interactWithItem()
    return 0

if __name__ == "__main__":
    main()