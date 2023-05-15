#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor)
from pybricks.parameters import Port, Stop, Button, Color
from pybricks.tools import wait
from pybricks.messaging import (BluetoothMailboxServer, BluetoothMailboxClient,
                                TextMailbox)

import os
import time

ev3 = EV3Brick()

claw_motor = Motor(Port.A)
vertical_motor = Motor(Port.B)
horizontal_motor = Motor(Port.C)

color_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)

CLAWOPENANGLE = 90.0

CLAWMAXVERTICALANGLE = 90.0

HORIZONTALMOTORMAXANGLE = 720.0
HORIZONTALMOTORHALFANGLE = 360.0

CLAWMAXHORIZONTALANGLE = 180.0


class Robot(object):
    """
    Base class for Robot
    """

    def __init__(self):
        self.robot_horizontal_motor_angle = 0.0
        self.robot_claw_motor_angle = 0.0
        self.robot_vertical_motor_angle = 0.0
        self.CLAWRESISTANCE = 0.0
        self.VERTICALRESISTANCE = 0.0
        self.rotation_scale = None
        self.zones = []
        self.color_dict = {}
        self.pickupzone = ()
        self.position_list = []
        self.count = 0

    def find_color(self):
        """
        A function that returns the name of the color sensed by the color
        sensor.\nReturns either Green/Blue/Red/Yellow
        """
        ev3.screen.clear()
        color = color_sensor.rgb()
        message = None
        if color[2] < 17:  # RED / GREEN / YELLOW
            if color[0] <= 6:
                message = "Green"
            else:  # RED / YELLOW
                if 0 <= color[1] <= 5:
                    message = "Red"
                else:
                    message = "Yellow"
        else:
            message = "Blue"
        ev3.screen.draw_text(20, 50, message)
        return message

    def calibrate(self):
        """
        A function that calibrate the robot's horizontal and vertical angles
        """
        RobotReset.reset_vertical(self)
        RobotReset.reset_all(self)

        claw_motor.run_until_stalled(100)
        claw_motor.reset_angle(0.0)
        self.robot_claw_motor_angle = 0.0

        RobotClaw.open_claw(self)
        self.CLAWRESISTANCE = RobotMotors.meassure_resistance(self, claw_motor)
        RobotClaw.close_claw(self)
        RobotClaw.open_claw(self)
        self.VERTICALRESISTANCE = RobotMotors.meassure_resistance(self, vertical_motor)

        RobotMotors.raise_arm(self)
        RobotClaw.close_claw(self)
        self.zones = Robot.calibrate_zones(self)
        self.pickupzone = self.zones[0]
        self.position_list = self.zones[1:]

    def calibrate_zones(self):
        """
        A function that allows the user to chose specific zones for
        drop-off and pickup using ev3 buttons
        """
        namelst = [("Pickup Zone", Color.RED),
                   ("Drop off Zone 1", Color.GREEN),
                   ("Drop off Zone 2", Color.YELLOW)]
        zonelst = []
        RobotClaw.open_claw(self)
        for i in namelst:
            ev3.light.on(i[1])
            ev3.screen.draw_text(20, 50, i[0])
            wait(500)
            while True:
                pressed_buttons = ev3.buttons.pressed()
                if len(pressed_buttons) > 0:
                    if pressed_buttons[0] == Button.CENTER:
                        break
                    if pressed_buttons[0] == Button.UP:
                        RobotMotors.move_by_given_motor_angle(self, vertical_motor, -10)
                    elif pressed_buttons[0] == Button.DOWN:
                        RobotMotors.move_by_given_motor_angle(self, vertical_motor, 10)
                    elif pressed_buttons[0] == Button.RIGHT:
                        RobotMotors.move_by_given_motor_angle(self, horizontal_motor, 10)
                    elif pressed_buttons[0] == Button.LEFT:
                        RobotMotors.move_by_given_motor_angle(self, horizontal_motor, -10)
            zonelst.append((RobotMotors.angle_to_degrees(self, self.robot_horizontal_motor_angle), self.robot_vertical_motor_angle))
            ev3.screen.clear()
        ev3.light.off()
        return zonelst

    def user_interface(self):
        """A function that allows testing of specific functions"""
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

            answer = input(": ")
            if answer == "1":
                RobotMotors.raise_arm(self)
            elif answer == "2":
                RobotMotors.lower_arm(self)
            elif answer == "3":
                RobotClaw.open_claw(self)
            elif answer == "4":
                RobotClaw.close_claw(self)
            elif answer == "5":
                print(Robot.find_color(self))
            elif answer == "6":
                motor = int(input("Motor (1 horizontal, 2 vertical): "))
                move_to_degree = float(input("Degree: "))
                if motor == 1:
                    RobotMotors.move_to_given_degree(self, horizontal_motor, move_to_degree)
                elif motor == 2:
                    RobotMotors.move_to_given_degree(self, vertical_motor, move_to_degree)
            elif answer == "7":
                is_holding = RobotClaw.is_holding_item(self)
                print("Item is being held: " + str(is_holding))
            elif answer == "8":
                RobotSorting.color_zone_sorting(self)
            elif answer == "0":
                RobotReset.exit_program(self)

    def manual(self):
        """
        A function that allows manual control of the robot
        """
        self.calibrate()
        self.user_interface()


class RobotMotors(Robot):
    """
    A class that handles motor movement
    """
    def angle_to_degrees(self, angle: float) -> float:
        """
        A function that converts motor angles to degrees
        """
        return angle / self.rotation_scale

    def degree_to_motor_angle(self, degree: float) -> float:
        """
        A function that converts degrees to motor angles
        """
        return degree * self.rotation_scale

    def move_by_given_motor_angle(self, motor, motor_angle: float, speed: int = 200):
        """
        Raise claw to sensor height and then move by the specified motor angle.
        [Required] motor
        [Required] motor angle
        [Optional] speed
        """

        if motor == horizontal_motor:
            RobotMotors.raise_arm(self, -90.0)
            self.robot_horizontal_motor_angle += motor_angle
        elif motor == vertical_motor:
            self.robot_vertical_motor_angle += motor_angle
        elif motor == claw_motor:
            self.robot_claw_motor_angle += motor_angle
        motor.run_angle(speed, motor_angle)

    def move_by_given_degree(self, motor, motor_degree: float, speed: int = 200):
        """
        Raise claw to sensor height and then convert input degrees to a motor
        angle, then move the horizontal motor by the motor angle.
        [Required] motor
        [Required] degree
        [Optional] speed
        """
        motor_angle = RobotMotors.degree_to_motor_angle(self, motor_degree)

        if motor == horizontal_motor:
            RobotMotors.raise_arm(self)
            self.robot_horizontal_motor_angle += motor_angle
        elif motor == vertical_motor:
            self.robot_vertical_motor_angle += motor_angle
        elif motor == claw_motor:
            self.robot_claw_motor_angle += motor_angle
        motor.run_angle(speed, motor_angle)

    def move_to_given_degree(self, motor, motor_degree: float, speed: int = 200):
        """
        Raise claw to sensor height and then convert input degrees to a motor
        angle, then reset the motor to its origin (0 degrees) and rotate to
        the new degrees, degrees is between -90 to 90 degrees from the origin.
        [Required] motor
        [Required] degree from origin
        [Optional] speed
        """
        if motor == horizontal_motor:
            RobotMotors.raise_arm(self, -90)
        motor.run_target(150, 0.0)
        if motor == horizontal_motor:
            self.robot_horizontal_motor_angle = 0.0
        elif motor == vertical_motor:
            self.robot_vertical_motor_angle = 0.0
        elif motor == claw_motor:
            self.robot_claw_motor_angle = 0.0
        move_motor_angle = RobotMotors.degree_to_motor_angle(self, motor_degree)

        RobotMotors.move_by_given_motor_angle(self, motor, move_motor_angle, speed)

    def meassure_resistance(self, motor):
        """
        Meassure resistance of a motor by running the motor until stalled at a
        angle of atleast 80%
        """
        resistance = motor.run_until_stalled(70, Stop.HOLD, 80)
        return resistance

    def raise_arm(self, offset=0.0):
        """
        Raise claw on the vertical axis to motor angle of the color sensor
        """
        self.robot_vertical_motor_angle = -240 + offset
        vertical_motor.run_target(100, -240.0 + offset)

    def lower_arm(self, offset=0.0):
        """
        Lower claw on vertical axis to default position after calibration
        """
        self.robot_vertical_motor_angle = offset
        vertical_motor.run_target(100, offset)


class RobotClaw(Robot):
    """
    A class that handles claw movement
    """
    def open_claw(self):
        """
        Open claw if the value is not bigger than the max claw motor angle
        """
        self.robot_claw_motor_angle = -CLAWOPENANGLE
        claw_motor.stop()
        claw_motor.run_target(200, -CLAWOPENANGLE)

    def close_claw(self):
        """
        Close claw by running the claw motor until stalled and then resetting
        the angle to 0
        """
        is_holding = RobotClaw.is_holding_item(self)
        if not is_holding:
            claw_motor.stop()
            claw_motor.run_target(200, 0.0)
            self.robot_claw_motor_angle = 0.0
            claw_motor.reset_angle(0.0)
        return is_holding

    def is_holding_item(self) -> bool:
        """
        A function that detects whether the clas is holding anything or not\n
        Returns true if holding anything
        """
        is_holding = False
        t_end = time.time() + 4.0
        while not (claw_motor.stalled() and claw_motor.angle() != 0.0):
            print("ANGLE: " + str(claw_motor.angle()))
            if time.time() >= t_end:
                if claw_motor.angle() <= -17.0:
                    is_holding = True
                break
            claw_motor.run_time(200, 100, Stop.HOLD)
        return is_holding


class RobotReset(Robot):
    """
    A class that handle reset functions for the robot
    """
    def reset_horizontal(self):
        """
        A function that resets the horizontal rotation
        """
        while not touch_sensor.pressed():
            RobotMotors.move_by_given_motor_angle(self, horizontal_motor, 5.0, 500)
        self.rotation_scale = self.robot_horizontal_motor_angle / 90.0
        RobotMotors.move_by_given_motor_angle(self, horizontal_motor, -(90.0 * self.rotation_scale))
        self.robot_horizontal_motor_angle = 0.0
        horizontal_motor.reset_angle(0.0)

    def reset_vertical(self):
        """
        A function that resets the vertical rotation
        """
        vertical_motor.run_until_stalled(100, Stop.HOLD, 10)
        self.robot_vertical_motor_angle = 0.0
        vertical_motor.reset_angle(0.0)

    def reset_all(self):
        """
        A function that resets both horizontal and vertical rotations
        """
        RobotClaw.close_claw(self)
        RobotReset.reset_horizontal(self)
        RobotReset.reset_vertical(self)

    def exit_program(self):
        """
        A function that call reset vertical then exits the program
        """
        RobotClaw.close_claw(self)
        RobotMotors.move_to_given_degree(self, horizontal_motor, 0.0)
        RobotReset.reset_vertical(self)
        os._exit(0)


class RobotSorting(Robot):
    """A class that handles sorting of colors"""
    def color_zone_sorting(self):
        """A function that collect a cube from the selected pickup zone from
        the calibration sequence.\n
        Then it assign a drop-off zone to that specific color for the duration
        of the run
        """
        color = None
        RobotMotors.raise_arm(self, -90.0)
        RobotMotors.move_to_given_degree(self, horizontal_motor, self.pickupzone[0])
        RobotClaw.open_claw(self)
        RobotMotors.lower_arm(self, self.pickupzone[1])
        wait(3000)
        holding = RobotClaw.close_claw(self)
        print("Holding: " + str(holding))
        if holding:
            RobotMotors.raise_arm(self)
            color = Robot.findColor(self)
            RobotMotors.raise_arm(self, -90)

            if (color not in self.color_dict and self.count < 2 and
                    color not in ["Blue", "Red", "Green", "Yellow"]):

                self.color_dict[color] = self.position_list[self.count]
                self.count += 1
            if color in self.color_dict:
                position = self.color_dict[color]
                RobotMotors.move_to_given_degree(self, horizontal_motor, position[0])
                RobotMotors.lower_arm(self, position[1])
                RobotClaw.open_claw(self)
            else:
                RobotMotors.lower_arm(self, self.pickupzone[1])
                RobotClaw.open_claw(self)

        RobotMotors.move_to_given_degree(self, horizontal_motor, 0.0)
        RobotMotors.raise_arm(self, -90)
        return color


class Communication(Robot):
    """
    A Communication base class
    """
    def __init__(self):
        self.SERVER = "ev3dev"


class Server(Communication):
    """
    A class that acts as a server in a bluetooth connection between two robots
    """
    def automate(self):
        """
        A function that automate the sorting of colors between two robots
        """
        robot = Robot()
        robot.calibrate()
        server = BluetoothMailboxServer()
        server.wait_for_connection()
        print("Connected")
        mbox = TextMailbox("Mail", server)
        SERVERPICKUP = "serverpickup"
        CLIENTPICKUP = "clientpickup"
        RobotMotors.move_to_given_degree(robot, horizontal_motor, 0.0)
        mbox.send(CLIENTPICKUP)
        while True:
            mbox.wait()
            if mbox.read() == SERVERPICKUP:
                RobotSorting.colorZoneSorting(robot)
                mbox.send(CLIENTPICKUP)


class Client(Communication):
    """
    A class that acts as a client in a bluetooth connection between two robots
    """
    def automate(self):
        """
        A function that automate the sorting between two robots
        """
        robot = Robot()
        robot.calibrate()
        client = BluetoothMailboxClient()
        client.connect(self.SERVER)
        mbox = TextMailbox("Mail", client)
        SERVERPICKUP = "serverpickup"
        CLIENTPICKUP = "clientpickup"
        RobotMotors.move_to_given_degree(robot, horizontal_motor, 0.0)

        while True:
            if mbox.read() == CLIENTPICKUP:
                pickupcolor = RobotSorting.colorZoneSorting(robot)
                if pickupcolor is None:
                    pass
                elif pickupcolor not in robot.color_dict:
                    mbox.send(SERVERPICKUP)
                    mbox.wait()


def main():
    """
    A function that everything starts from
    """
    server = Server()
    server.automate()
    # client = Client()
    # client.automate()


if __name__ == "__main__":
    main()
