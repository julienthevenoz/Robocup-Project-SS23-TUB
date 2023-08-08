# -*- encoding: UTF-8 -*-
""" Say 'hello, you' each time a human face is detected

"""

import sys
import time

import math
import almath

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

from Graphs import GridGraph
    
NAO_IP = "nao3.local"

# distance and angle of the engagement zones
zone1Distance = 1.0
zone2Distance = 3.0
limitAngle = 90.0

# parameters how nao moves, so he is stable and doesn't fall
moveConfig = [["MaxStepX", 0.02], ["MaxStepFrequency", 0.2], ["StepHeight", 0.03]]

# size of an individual step in grid
stepSize = 0.3

class PeopleTrackingModule(ALModule):   
    """ 
    This class implements a robot, that turns around
    until he sees a human face, and then goes into 
    that direction avoiding obstacles until the human
    is in zone 1

    """
    def __init__(self, name):
        """
        Initializes the robot
        """

        # ALModule superclass initialize
        ALModule.__init__(self, name)
    
        # instantiate all necessary proxys
        self.tts = ALProxy("ALTextToSpeech")
        self.motion = ALProxy("ALMotion")
        self.peoplePerception = ALProxy("ALPeoplePerception")
        self.awareness = ALProxy("ALBasicAwareness")
        self.memory = ALProxy("ALMemory")
        self.ez = ALProxy("ALEngagementZones")
        self.sonarProxy = ALProxy("ALSonar")

        self.motion.wakeUp()
        self.motion.setStiffnesses("body", 1)

        # intialize other variables
        self.peopleInZone1 = False
        self.sonarLeftDetected = False
        self.peopleInZone1to3 = False
        self.sonarRightDetected = False
        self.obstacleDetected = False
        self.prevPeopleInZone1 = False
        self.prevPeopleInZone1to3 = False

        # for obstacle avoidance
        self.Graph = None
        self.actions = []
        self.currPos = None
        self.orientation = "north"

        #to subscribe I need to give : eventname,module that will be
        #called, function that will be called
        self.memory.subscribeToEvent("PeoplePerception/PeopleDetected",
                                "PeopleTracker",
                                "onPeopleDetected")
        self.memory.subscribeToEvent("EngagementZones/PeopleInZonesUpdated",
                                "PeopleTracker",
                                "onPeopleInZonesUpdated")
        self.memory.subscribeToEvent("SonarLeftDetected", 
                                "PeopleTracker", 
                                "onSonarLeftDetected")
        self.memory.subscribeToEvent("SonarRightDetected", 
                                "PeopleTracker", 
                                "onSonarRightDetected")
        self.memory.subscribeToEvent("SonarLeftNothingDetected", 
                                "PeopleTracker", 
                                "onSonarLeftNothingDetected")
        self.memory.subscribeToEvent("SonarRightNothingDetected", 
                                "PeopleTracker", 
                                "onSonarRightNothingDetected")

        # subscribe to sonar sensor
        self.sonarProxy.subscribe("PeopleTracker")
        
        # Intialize distances of engagement zones
        self.ez.setSecondLimitDistance(zone2Distance)
        self.ez.setFirstLimitDistance(zone1Distance)
        self.ez.setLimitAngle(limitAngle)

        # initialize engagement zones
        self.awareness.setEngagementMode("FullyEngaged")
        self.awareness.setTrackingMode("Head")
        self.awareness.startAwareness()

    def onSonarLeftDetected(self):
        """
        This method is called when the left sonar detects sth
        """
        self.sonarLeftDetected = True
        self.obstacleDetected = True

    def onSonarRightDetected(self):
        """
        This method is called when the right sonar detects sth
        """
        self.sonarRightDetected = True
        self.obstacleDetected = True

    def onSonarLeftNothingDetected(self):
        """
        This method is called when the left sonar detects nth
        """
        self.sonarLeftDetected = False
        self.obstacleDetected = self.sonarRightDetected

    def onSonarRightNothingDetected(self):
        """
        This method is called when the right sonar detects nth
        """
        self.sonarRightDetected = False 
        self.obstacleDetected = self.sonarRightDetected

    def onPeopleDetected(self):
        """
        This method is called when someone is being detected
        """
        print("People Detected")

    def onPeopleInZonesUpdated(self):
        """ 
        Sets peopleInZone1 to true if there are people in Zone 1 
        """

        print("Zone 1: " + str(self.memory.getData("EngagementZones/PeopleInZone1")))
        print("Zone 2: " + str(self.memory.getData("EngagementZones/PeopleInZone2")))
        print("Zone 3: " + str(self.memory.getData("EngagementZones/PeopleInZone3")))

        # number of people in each zone
        numPeopleInZone1 = len(self.memory.getData("EngagementZones/PeopleInZone1"))
        numPeopleInZone2 = len(self.memory.getData("EngagementZones/PeopleInZone2"))
        numPeopleInZone3 = len(self.memory.getData("EngagementZones/PeopleInZone3"))

        # are there any people in zone 1 to 3
        self.peopleInZone1to3 = numPeopleInZone1 + numPeopleInZone2 + numPeopleInZone3 > 0

        # has someone ever been in zone 1 to 3
        if self.peopleInZone1to3:
            self.prevPeopleInZone1to3 = True

        # are there currently an people in zone1
        self.peopleInZone1 = False if numPeopleInZone1 == 0 else True

        # has someone ever been in zone 1
        if numPeopleInZone1 > 0:
            self.prevPeopleInZone1 = True

    def movePitch(self, angle):
        """
        moves the HeadPitch joint into angle position
        """
        name = "HeadPitch"
        angles = angle*almath.TO_RAD
        fractionMaxSpeed = 0.1
        self.motion.setAngles(name,angles,fractionMaxSpeed)

    def moveYaw(self, angle):
        """
        moves the HeadYaw joint into angle position
        """
        name = "HeadYaw"
        angles = angle*almath.TO_RAD
        fractionMaxSpeed = 0.1
        self.motion.setAngles(name,angles,fractionMaxSpeed)

    def moveForward(self)  :
        """ 
        Makes the robot move forward 
        """
        self.motion.moveTo(stepSize,0,0, moveConfig)

    def moveToHuman(self)  :
        """ 
        Makes nao move forward until it detects a human in zone1
        avoiding obstacles
        """
        
        # mapping direction strings to angles
        anglesTurn = {  "north": 0,
                        "east": -math.pi/2,
                        "south": math.pi,
                        "west": math.pi/2
        }

        # relative positions of the nodes in the graph in that direction
        # used to update current position
        relativePositions = {   "north": (0, -1),
                                "east": (1, 0),
                                "south": (0, 1),
                                "west": (-1,0)}

        # maps directions to position of obstacles
        # used to update graph when obstacle detected
        # every obstacle is a 3x2 square in the graph
        relativePositionsObstacles = {"north": [(0,-1), (-1, -1), (1, -1), (0, -2), (-1, -2), (1, -2)],
                                "east": [(1, 0), (1, -1), (1, 1), (2, 0), (2, -1), (2, 1)],
                                "south": [(0,1), (1, 1), (-1, 1), (0, 2), (-1, 2), (1, 2)],
                                "west": [(-1,0), (-1, 1), (-1,-1), (-2,0), (-2, -1), (-2, 1)]
                                }

        # Intialize the grid and run Breadth-First-Search
        self.Graph = GridGraph((15,15))
        self.Graph.bfs((7,14))
        self.prevPeopleInZone1 = False
        self.actions = self.Graph.movesTo((7,0))
        
        # initialize variables
        currPos = (7,14)
        goal = (7,0)
        i = 0
        steps = 15
        time.sleep(1.5)

        # the actual main loop
        while self.actions and not self.prevPeopleInZone1 and i < 15 and self.Graph.hasPathTo(goal):
            
            # turn into the next direction
            action = self.actions.pop(0)
            self.motion.moveTo(0,0, anglesTurn[action] - anglesTurn[self.orientation], moveConfig)
            self.orientation = action

            rPos = relativePositions[action]
            # when an obstacle is detected
            if self.obstacleDetected:

                # make nao say obstacle
                self.tts.say("obstacle")
                # remove obstacles from graph
                for pos in relativePositionsObstacles[action]:
                    self.Graph.remove((currPos[0] + pos[0], currPos[1] + pos[1]))
                # get new path
                self.Graph.bfs(currPos)
                # update actions
                self.actions = self.Graph.movesTo((7,0))

            else:
                # make move and update position
                self.moveForward()
                # make nao look up
                if not self.peopleInZone1to3:
                    self.movePitch(-38.0)
                currPos = (currPos[0] + rPos[0], currPos[1] + rPos[1])
            
            self.Graph.setPosition(currPos)
            print(self.Graph)
            i += 1

        # if nao finds a human
        if self.prevPeopleInZone1:
            print("hello there")
            self.tts.say("hello there")  
        # if nao didnt find anyone start again
        else:
            self.lookForHuman() 

    def lookForHuman(self)  :
        """
        makes nao turn until he sees a human
        """
        self.movePitch(-30.0)
        for i in range(10):
            self.movePitch(-38.0)
            time.sleep(2.5)
            if self.prevPeopleInZone1to3:
                break
            self.motion.moveTo(0,0, math.pi/5, moveConfig)

        # when a human is detected, NAO turns its head in his direction. By making NAOs body
        #turn at the same angle than its head, the whole body faces the human
        yaw = self.memory.getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value")
        self.motion.moveTo(0,0, yaw, moveConfig)
        self.moveToHuman()


def main():
    """ 
    Main entry point
    Some parts are from the Nao-Documentation
    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       pip,         # parent broker IP
       pport)       # parent broker port

    # stand nao up
    global posturemodule
    posturemodule = ALProxy("ALRobotPosture", NAO_IP, 9559)
    posturemodule.goToPosture("StandInit", 0.8) 

    global PeopleTracker
    PeopleTracker = PeopleTrackingModule("PeopleTracker")
    PeopleTracker.lookForHuman()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        PeopleTracker.motion.rest()
        print "Shutting Down"
        myBroker.shutdown()



if __name__ == "__main__":
    main()
