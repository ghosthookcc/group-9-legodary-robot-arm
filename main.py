#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, TextMailbox, LogicMailbox

from threading import Thread
from enum import Enum

import sys
import os
import time
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
    class States(Enum):
        IDLE = 0
        WORKING = 1
        EXITING = 2

    def __init__(self):
        self.robotHorizontalMotorAngle = 0.0 
        self.robotClawMotorAngle = 0.0
        self.robotVerticalMotorAngle = 0.0
        self.CLAWRESISTANCE = 0.0
        self.VERTICALRESISTANCE = 0.0
        self.rotationScale = None
        self.state = self.States.IDLE
        self.zones = []
        self.colorDict = {}
        self.pickupzone = ()
        self.positionList = []
        self.count = 0
        
    def findColor(self):
        ev3.screen.clear()
        color = colorSensor.color()
        message = None
        if color == Color.BLUE:
            message = "Blue"
        elif color == Color.RED:
            message = "Red"
        elif color == Color.GREEN:
            message = "Green"
        elif color == Color.YELLOW:
            message == "Yellow"
        ev3.screen.draw_text(20,50,message)
        return color
    

    def calibrate(self):
        RobotReset.resetVertical(self)
        RobotReset.resetAll(self)

        clawMotor.run_until_stalled(100)
        clawMotor.reset_angle(0.0)
        self.robotClawMotorAngle = 0.0

        RobotClaw.openClaw(self)
        self.CLAWRESISTANCE = RobotMotors.meassureResistance(self, clawMotor)
        RobotClaw.closeClaw(self)
        RobotClaw.openClaw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassureResistance(self, verticalMotor)

        RobotClaw.raiseClaw(self)
        RobotClaw.closeClaw(self)
        self.zones = Robot.calibrateZones(self)
        self.pickupzone = self.zones[0]
        self.positionList = self.zones[1:]
    
    def calibrateZones(self):
        namelst = [("Pickup Zone",Color.RED),( "Drop off Zone 1",Color.GREEN),( "Drop off Zone 2",Color.YELLOW) ]
        zonelst = []
        RobotClaw.openClaw(self)
        for i in namelst:
            ev3.light.on(i[1])
            ev3.screen.draw_text(20, 50, i[0])
            wait(500)
            while True:
                pressedButtons = ev3.buttons.pressed()
                if len(pressedButtons) > 0:
                    if pressedButtons[0] == Button.CENTER:
                        break
                    elif pressedButtons[0] == Button.UP:
                        RobotMotors.moveByGivenMotorAngle(self,verticalMotor, -10)
                    elif pressedButtons[0] == Button.DOWN:
                        RobotMotors.moveByGivenMotorAngle(self,verticalMotor, 10)
                    elif pressedButtons[0] == Button.RIGHT:
                        RobotMotors.moveByGivenMotorAngle(self,horizontalMotor, 10)
                    elif pressedButtons[0] == Button.LEFT:
                        RobotMotors.moveByGivenMotorAngle(self,horizontalMotor, -10)
            zonelst.append((RobotMotors.angleToDegrees(self,self.robotHorizontalMotorAngle), verticalMotor.angle()))
            ev3.screen.clear()
        ev3.light.off()
        return zonelst

    def userInterface(self):
        while True:
            input(": Press enter :")
            os.system("clear")
            print("1: Raise Claw")
            print("2: Lower Claw")
            print("3: Open Claw")
            print("4: Close Claw")
            print("5: Find color")
            print("6: Move to degree")
            print("7: Check if a item is being held")
            print("8: Sort Color")
            print("0: Exit")

            #sorc = input("1 for server, 2 for client: ")
            #if sorc == "1":
            #    network = Server()
            #elif sorc == "2":
            #    network = Client()
            #else:
            #    print("No")

            #network.initiate(network)

            #while True:
            
             #   network.currentState
              #  wait(5000)
            
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
            elif answer == "8":
                RobotSorting.colorZoneSorting(self)
            elif answer == "0":
                RobotReset.exitProgram(self)
    
    def manual(self):
        self.calibrate()
        self.userInterface()

    def automate(self):
        self.calibrate()
        while True:
            wait(5000)
            RobotSorting.colorZoneSorting(self)
            

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
        elif (motor == clawMotor):
            self.robotClawMotorAngle += motorAngle
        motor.run_angle(speed, motorAngle)
    def moveByGivenDegree(self, motor, motorDegree: float, speed: int = 200):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then move the horizontal motor by the motor angle.
        [Required] motor
        [Required] degree
        [Optional] speed
        """
        motorAngle = RobotMotors.degreeToMotorAngle(self, motorDegree)

        if (motor == horizontalMotor):
            #RobotClaw.raiseClaw(self)
            self.robotHorizontalMotorAngle += motorAngle
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle += motorAngle
        elif (motor == clawMotor):
            self.robotClawMotorAngle += motorAngle
        motor.run_angle(speed, motorAngle)
    def moveToGivenDegree(self, motor, motorDegree: float, speed: int = 200):
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then reset the motor to its origin (0 degrees) and rotate to the new degrees,
        degrees is between -90 to 90 degrees from the origin.
        [Required] motor
        [Required] degree from origin
        [Optional] speed
        """

        if (motor == horizontalMotor):
            motor.run_target(150, 0.0)
            self.robotHorizontalMotorAngle = 0.0
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle = 0.0
        elif (motor == clawMotor):
            motor.run_target(150, 0.0)
            self.robotClawMotorAngle = 0.0
        moveMotorAngle = RobotMotors.degreeToMotorAngle(self, motorDegree)

        RobotMotors.moveByGivenMotorAngle(self, motor, moveMotorAngle, speed)
    
    def meassureResistance(self, motor):
        """
        Meassure resistance of a motor by running the motor until stalled at a angle of atleast 80%
        """
        resistance = motor.run_until_stalled(70,Stop.HOLD,80)
        return resistance
    
class RobotClaw(Robot):
    def raiseClaw(self, offset = 0.0):
        """
        Raise claw on the vertical axis to motor angle of the color sensor
        """
        self.robotVerticalMotorAngle = -230 + offset
        verticalMotor.run_target(100, -230.0 + offset)

    def lowerClaw(self, offset = 0.0):
        """
        Lower claw on vertical axis to default position after calibration
        """
        self.robotVerticalMotorAngle = offset
        verticalMotor.run_target(100, offset)

    def openClaw(self):
        """
        Open claw if the value is not bigger than the max claw motor angle
        """
        self.robotClawMotorAngle = -CLAWOPENANGLE
        clawMotor.stop()
        clawMotor.run_target(200, -CLAWOPENANGLE)

    def closeClaw(self):
        """
        Close claw by running the claw motor until stalled and then resetting the angle to 0
        """
        isHolding = RobotClaw.isHoldingItem(self)
        if not isHolding:
            clawMotor.stop()
            clawMotor.run_target(200, 0.0)
            self.robotClawMotorAngle = 0.0
            clawMotor.reset_angle(0.0)
        return isHolding

    def isHoldingItem(self) -> bool:
        isHolding = False
        t_end = time.time() + 3.0
        while not (clawMotor.stalled() and clawMotor.angle() != 0.0):
            print("ANGLE: " + str(clawMotor.angle()))
            if (time.time() > t_end):
                if (clawMotor.angle() <= -16.0):
                    isHolding = True
                break
            clawMotor.run_time(200, 100, Stop.HOLD)
        return isHolding

    def pickupItem(self):
        RobotClaw.openClaw(self)
        wait(1000)
        RobotClaw.lowerClaw(self)
        wait(1000)
        RobotClaw.closeClaw(self)
        wait(1000)
        RobotClaw.raiseClaw(self)
        wait(1000)

    def dropOffItem(self):
        RobotClaw.lowerClaw(self)
        wait(1000)
        RobotClaw.openClaw(self)
        wait(1000)
        RobotClaw.raiseClaw(self)
        wait(1000)

class RobotReset(Robot):
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
        RobotReset.resetHorizontal(self)
        RobotReset.resetVertical(self)

    def exitProgram(self):
        self.state = self.States.EXITING
        RobotClaw.closeClaw(self)
        RobotMotors.moveToGivenDegree(self, horizontalMotor, 0.0)
        RobotReset.resetVertical(self)
        os._exit(0)
    
class RobotSorting(Robot):
    def colorZoneSorting(self):
        if (self.state == self.States.WORKING):
            RobotClaw.raiseClaw(self, -90.0)
            print(self.pickupzone)
            print(self.positionList)
            RobotMotors.moveToGivenDegree(self,horizontalMotor,self.pickupzone[0]) 
            RobotClaw.lowerClaw(self, self.pickupzone[1])
            RobotClaw.openClaw(self)
            holding = RobotClaw.closeClaw(self)
            RobotClaw.raiseClaw(self, -90.0)
            print("Holding: " + str(holding))
            if holding:
                RobotClaw.raiseClaw(self)
                color = Robot.findColor(self)
                print(color)
        
                if color not in self.colorDict and self.count<2:
                    self.colorDict[color] = self.positionList[self.count]  #+1 för första element är pickupZone
                    self.count +=1
                if color in self.colorDict:
                    position = self.colorDict[color]
                    print(position)
                    RobotMotors.moveToGivenDegree(self,horizontalMotor,position[0])
                    RobotClaw.lowerClaw(self,position[1])
                    RobotClaw.openClaw(self)
                else:
                    RobotMotors.moveToGivenDegree(self,horizontalMotor,self.pickupzone[0])
                    RobotClaw.lowerClaw(self,self.pickupzone[1]) # TEST
                    RobotClaw.openClaw(self)
            else:
                RobotMotors.lowerClaw(self, -90.0)
                RobotMotors.moveToGivenDegree(self, horizontalMotor, 0.0)
        RobotClaw.raiseClaw(self, -90)
        self.state = self.States.IDLE

class Communication(Robot): #håll koll på vilket stadie roboten är i just nu
    Instance = None
    def __init__(self, connectionName):
        self.connectionName = connectionName
        self.mbox = None
        self.lbox = None
        self.initiate(self) # om initiate inte kör alla 3 iterationer av initiate kan ni ta bort kommentaren i init koden i deriverade klasser

    def main_loop(self):
        while (self.state != self.States.EXITING):
            self.UpdateState(self)
            time.sleep(0.5)

    def getInstance(self):
        if (self.Instance == None):
            self.Instance = self
        return self.Instance

    def initiate(self):
        self.getInstance()
        self.lbox = LogicMailbox("CurrentState",self.Instance)

    def UpdateState(self) -> bool:
        isReady = True
        if (self.state == self.States.WORKING):
            isReady = False
        if (self.state == self.States.IDLE):
            self.lbox.send(True)
        if (self.state == self.States.IDLE and self.lbox.read() == True):
            self.state = self.States.WORKING
        return isReady

class Server(Communication):
    def __init__(self):
        Communication.__init__(connectionName="ev3dev")
        #self.initiate(self)

    def initiate(self):
        super().initiate()
        self.Instance = BluetoothMailboxServer()       
        self.mbox = TextMailbox(self.connectionName, self.Instance)
        print("[/] Waiting for connection...")
        self.Instance.wait_for_connection()
        print("[+] Connected!")
        self.mbox.wait()
        print(self.mbox.read())
        self.mbox.send("[+] hello to you!")
    
class Client(Communication):
    def __init__(self):
        Communication.__init__(connectionName="ev3client")
        #self.initiate(self)

    def initiate(self):
        super().initiate()
        self.Instance = BluetoothMailboxClient()
        mbox = TextMailbox("ev3client", self.Instance)
        print('[/] establishing connection...')
        self.Instance.connect(self.connectionName)
        print('[+] Connected!')
        self.mbox.send('[+] hello!')
        self.mbox.wait()
        print(self.mbox.read())
    
robot = Robot()

def main():
    server = Server()
    
    communicationMainLoopThread = Thread(target=server.main_loop)
    communicationMainLoopThread.daemon = False
    communicationMainLoopThread.setName("StateObserver")
    communicationMainLoopThread.start() 
    
    robot.automate()

if __name__ == "__main__":
    main()
