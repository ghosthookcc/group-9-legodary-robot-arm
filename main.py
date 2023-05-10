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

import os
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
        PICKUP = 1
        SORTING = 2
        WAITING = 3
        EXITING = 4

    def __init__(self):
        self.robotHorizontalMotorAngle: float = 0.0 
        self.robotClawMotorAngle: float = 0.0
        self.robotVerticalMotorAngle: float = 0.0
        self.CLAWRESISTANCE: float = 0.0
        self.VERTICALRESISTANCE: float = 0.0
        self.rotationScale: float = 0.0
        self.state: Robot.States = self.States.IDLE
        self.zones: list = []
        self.colorDict: dict = {}
        self.pickupzone: tuple = ()
        self.positionList: list = []
        self.count: int = 0
        self.running: bool = False

    def findColor(self) -> Color:
        ev3.screen.clear()
        color: Color = colorSensor.color()
        message: str = None
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
    
    def calibrate(self) -> None:
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
    
    def calibrateZones(self) -> list:
        namelst: list = [("Pickup Zone",Color.RED),( "Drop off Zone 1",Color.GREEN),( "Drop off Zone 2",Color.YELLOW) ]
        zonelst: list = []
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

    def userInterface(self) -> None:
        while True:
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
            
            answer: int = input(": ")
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
    
    def manual(self) -> None:
        self.calibrate()
        self.running = True
        self.userInterface()

    def automate(self) -> None:
        self.calibrate()
        self.running = True
        while self.running:
            wait(5000)
            RobotSorting.colorZoneSorting(self)

    def setState(self, state: States) -> None:
        self.state = state
    def getState(self) -> States:
        return self.state
    def isRunning(self) -> bool:
        return self.running

class RobotMotors(Robot):
    def angleToDegrees(self, angle: float) -> float:
        return angle / self.rotationScale
    def degreeToMotorAngle(self, degree: float) -> float:
        return degree * self.rotationScale
    def moveByGivenMotorAngle(self, motor, motorAngle: float, speed: int = 200) -> None:
        """
        Raise claw to sensor height and then move by the specified motor angle.
        [Required] motor
        [Required] motor angle
        [Optional] speed
        """

        if (motor == horizontalMotor):
            #RobotClaw.raiseClaw(self) Kanske behöver den här fortfarande, testa med och utan för att se skillnad
            self.robotHorizontalMotorAngle += motorAngle
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle += motorAngle
        elif (motor == clawMotor):
            self.robotClawMotorAngle += motorAngle
        motor.run_angle(speed, motorAngle)
    def moveByGivenDegree(self, motor, motorDegree: float, speed: int = 200) -> None:
        """
        Raise claw to sensor height and then convert input degrees to a motor angle,
        then move the horizontal motor by the motor angle.
        [Required] motor
        [Required] degree
        [Optional] speed
        """
        motorAngle = RobotMotors.degreeToMotorAngle(self, motorDegree)

        if (motor == horizontalMotor):
            self.robotHorizontalMotorAngle += motorAngle
        elif (motor == verticalMotor):
            self.robotVerticalMotorAngle += motorAngle
        elif (motor == clawMotor):
            self.robotClawMotorAngle += motorAngle
        motor.run_angle(speed, motorAngle)
    def moveToGivenDegree(self, motor, motorDegree: float, speed: int = 200) -> None:
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
    
    def meassureResistance(self, motor: Motor) -> float:
        """
        Meassure resistance of a motor by running the motor until stalled at a angle of atleast 80%
        """
        resistance = motor.run_until_stalled(70,Stop.HOLD,80)
        return resistance
    
class RobotClaw(Robot):
    def raiseClaw(self, offset: float = 0.0) -> None:
        """
        Raise claw on the vertical axis to motor angle of the color sensor
        """
        self.robotVerticalMotorAngle = -230 + offset
        verticalMotor.run_target(100, -230.0 + offset)

    def lowerClaw(self, offset: float = 0.0) -> None:
        """
        Lower claw on vertical axis to default position after calibration
        """
        self.robotVerticalMotorAngle = offset
        verticalMotor.run_target(100, offset)

    def openClaw(self) -> None:
        """
        Open claw if the value is not bigger than the max claw motor angle
        """
        self.robotClawMotorAngle = -CLAWOPENANGLE
        clawMotor.stop()
        clawMotor.run_target(200, -CLAWOPENANGLE)

    def closeClaw(self) -> None:
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
            if (time.time() > t_end):
                if (clawMotor.angle() <= -16.0):
                    isHolding = True
                break
            clawMotor.run_time(200, 100, Stop.HOLD)
        return isHolding

class RobotReset(Robot):
    def resetHorizontal(self) -> None:
        while not touchSensor.pressed():
            RobotMotors.moveByGivenMotorAngle(self, horizontalMotor, 5.0, 500)
        self.rotationScale = self.robotHorizontalMotorAngle / 90.0
        RobotMotors.moveByGivenMotorAngle(self, horizontalMotor, -(90.0 * self.rotationScale))
        self.robotHorizontalMotorAngle = 0.0
        horizontalMotor.reset_angle(0.0)

    def resetVertical(self) -> None:
        verticalMotor.run_until_stalled(100, Stop.HOLD, 10)
        self.robotVerticalMotorAngle = 0.0
        verticalMotor.reset_angle(0.0)

    def resetAll(self) -> None:
        RobotClaw.closeClaw(self)
        RobotReset.resetHorizontal(self)
        RobotReset.resetVertical(self)

    def exitProgram(self) -> None:
        self.state = self.States.EXITING
        RobotClaw.closeClaw(self)
        RobotMotors.moveToGivenDegree(self, horizontalMotor, 0.0)
        RobotReset.resetVertical(self)
        self.running = False
        os._exit(0)
    
class RobotSorting(Robot):
    def colorZoneSorting(self) -> None:
        if (self.state == self.States.PICKUP):
            RobotClaw.raiseClaw(self, -90.0)
            RobotMotors.moveToGivenDegree(self,horizontalMotor,self.pickupzone[0]) 
            RobotClaw.lowerClaw(self, self.pickupzone[1])
            RobotClaw.openClaw(self)
            holding = RobotClaw.closeClaw(self)

            self.state = self.States.SORTING

            if holding:
                RobotClaw.raiseClaw(self)
                color = Robot.findColor(self)

                if color not in self.colorDict and self.count < 2:
                    self.colorDict[color] = self.positionList[self.count]
                    self.count +=1
                if color in self.colorDict:
                    position = self.colorDict[color]
                    RobotMotors.moveToGivenDegree(self,horizontalMotor,position[0])
                    RobotClaw.lowerClaw(self,position[1])
                    RobotClaw.openClaw(self)
            else:
                Communication.SendNewLogicState(self, False)
        RobotClaw.openClaw(self)
        RobotClaw.raiseClaw(self, -90.0)
        RobotMotors.moveToGivenDegree(self, horizontalMotor, 0.0)
        self.state = self.States.IDLE

class Communication(object):
    Instance: object = None
    reference: object = None
    def __init__(self, connectionName: str, reference: object):
        self.reference = reference
        self.connectionName: str = connectionName
        self.mbox: TextMailbox = None
        self.lbox: LogicMailbox = None
        self.initiate()

    def main_loop(self) -> None:
        while (self.reference.getState() != Robot.States.EXITING):
            Communication.UpdateState(self)
            time.sleep(0.2)

    def getInstance(self) -> None:
        return self.Instance

    def initiate(self) -> None:
        self.getInstance()
        self.lbox = LogicMailbox("CurrentState",self.Instance)

    def UpdateState(self) -> bool: #håll koll på vilket stadie roboten är i just nu
        isReady = True
        if self.reference.isRunning():
            currentState = self.reference.getState()
            if (self.lbox.read() == False): 
                self.reference.setState(self.States.WAITING)
            if (currentState == Robot.States.IDLE and self.lbox.read() == True):
                self.reference.setState(self.States.PICKUP)
            if (currentState == Robot.States.SORTING):
                self.lbox.send(True)
        return isReady
    
    def SendNewLogicState(self, state: bool) -> None:
        self.lbox.send(state)

class Server(Communication):
    def __init__(self, reference: object):
        Communication.__init__(self, "ev3dev", reference)
        Communication.initiate(self)

    def initiate(self) -> None:
        super().initiate()
        self.Instance = BluetoothMailboxServer()       
        self.mbox = TextMailbox(self.connectionName, self.Instance)
        print("[/] Waiting for connection...")
        self.Instance.wait_for_connection()
        print("[+] Connected!")
        self.mbox.wait()
        print(self.mbox.read())
        self.mbox.send("[+] hello to you!")

        self.lbox.wait()

class Client(Communication):
    def __init__(self, reference: object):
        Communication.__init__(self, "ev3client", reference)
        Communication.initiate(self)

    def initiate(self) -> None:
        super().initiate()
        self.Instance = BluetoothMailboxClient()
        self.mbox = TextMailbox(self.connectionName, self.Instance)
        print("[/] establishing connection...")
        self.Instance.connect(self.connectionName)
        print("[+] Connected!")
        self.mbox.send("[+] hello!")
        self.mbox.wait()
        print(self.mbox.read())

        Communication.SendNewLogicState(self, True)
    
robot: Robot = Robot()

def main():
    server: Server = Server(reference = robot)
    communicationMainLoopThread: Thread = Thread(target = server.main_loop, 
                                                 name   = "StateObserver",
                                                 daemon = False # Testa det här värdet med True också om server och robot inte kör parallelt
                                                )
    communicationMainLoopThread.start() 

    robot.automate()

if __name__ == "__main__":
    main()
