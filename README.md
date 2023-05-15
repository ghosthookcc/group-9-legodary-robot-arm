# Group 9 - PA1473 Project 2 Description 

## Introduction
- Using two remodeled versions of the Lego Mindstorms EV3 Robot Arm, this project makes the robots communicate with eachother to sort red, green, blue and yellow bricks into different zones.
- The robots use a Server and a Client to communicate and send messages to eachother as to not collide. Each robot sorts up to two colors in two zones and if a brick with a different color is found, that brick gets passed on to the other robot and lets that one try to sort it instead.

## Getting started
- If not already done:
	-Install Git from https://git-scm.com/downloads
	- Create a Github account and follow "https://github.com/ghosthookcc"
- Create a Trello account and join the following trello board from https://trello.com/b/cFRph969/legodary-robot-arm
- Install Python programming language
- In CMD do commands as administrator:
	- pip install pybricks
- Install Visual Studio Code (VSCode)
- Install VSCode extention LEGO Mindstorms EV3 MicroPython
- Clone the projects repository from https://github.com/ghosthookcc/group-9-legodary-robot-arm

## Building and running
- The program begins with calibrating the different positions and heights it's working within. It calibrates the degree of the different motors when the arm is touching the ground, calibrates the claws "open" and "closed" state, and afterwards makes the user manually decide where each used zone is located with the use of the buttons on the EV3 Brick. The amount of zones the robot saves is set to three, with the first being the pick-up zone and remaining two, the zones associated with the first colors introduced to the robots color sensor.
- After the startup procedure, the robot periodically checks for a new lego-brick placed in the pickup-zone by "feeling" if a brick is in the pick-up zone. If there is then pick it up, check the color of it and assign it a zone and place it there. If however all the zones already are assigned and the current brick has a color that doesn't match the color of an assigned zone, then communicate with the other robot to pick up the brick instead and to go through the same procedure of sorting it.

- Automation between two Robots
	- Both Robots has to be named "ev3dev" and been connected via bluetooth
	- If there's an automation between two robots, two separate files need to be downloaded into the separate robots.
		- Client robot need to contain the main.py file. In the file the main function has to contain two separate calls:
			- First: client = Client()
			- Second: client.automate()
		- Server robot need to contain the main.py file. In the file the main function has to contain two separate calls:
			- First: server = Server()
			- Second: server.automate()
	- The server robot needs to do its calibration sequence before the client robot can complete is calibration sequence.


## Features
Checklist containing the user stories given by the customer, [x] meaning they were completed within the time frame of the project.

- [x] US_1: As a customer, I want the robot to pick up items from a designated position.
- [x] US_2: As a customer, I want the robot to drop items off at a designated position.
- [x] US_3: As a customer, I want the robot to be able to determine if an item is present at a given location.
- [x] US_4: As a customer, I want the robot to tell me the color of an item at a designated position.
- [x] US_5: As a customer, I want the robot to drop items off at different locations based on the color of the item.
- [x] US_6: As a customer, I want the robot to be able to pick up items from elevated positions.
- [x] US_8: As a customer, I want to be able to calibrate items with three different colors and drop the items off at specific drop-off zones based on color.
- [x] US_9: As a customer, I want the robot to check the pickup location periodically to see if a new item has arrived.
- [x] US_10: As a customer, I want the robots to sort items at a specific time.
- [x] US_11: As a customer, I want two robots to communicate and work together on items sorting without colliding with each other.
- [x] US_12: As a customer, I want to be able to manually set the locations and heights of one pick-up zone and two drop-off zones. 


### Credits
@ghosthookcc<br/>
@Adam-Mattsson<br/>
@Xzizide<br/>
@EdvardJae<br/>
