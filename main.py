
class RobotSorting(Robot):
    colorDict = {}
    positionList = [90,0,-45,-90]   #dessa värden kan ändras till olika positions
    count = 0
    
    def colorZoneSorting(self):
        horizontalMotor.run_target(200,RobotSorting.positionList[0])
        RobotClaw.pickupItem(self)
        color = Robot.findColor(self)
    
        if color not in RobotSorting.colorDict and RobotSorting.count<3:
            RobotSorting.colorDict[color] = RobotSorting.positionList[RobotSorting.count+1]  #+1 för första element är pickupZone
            RobotSorting.count +=1
        if color in RobotSorting.colorDict:
            position = RobotSorting.colorDict[color]
            horizontalMotor.run_target(200,position)
 
        RobotClaw.dropOffItem(self)
