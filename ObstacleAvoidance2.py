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
from position import Position

NAO_IP = "nao3.local"

zone1_distance = 1.5
zone2_distance = 3.0
limit_angle = 90.0

step_size = 0.25

class SoundLocaterModule(ALModule):   
    """ This class implements a robot, that goes into 
    a specific Direction avoiding obst

    """
    def __init__(self, name):
        ALModule.__init__(self, name)
    
        # instantiate all necessary proxys
        self.tts = ALProxy("ALTextToSpeech")
        self.motion = ALProxy("ALMotion")
        self.peoplePerception = ALProxy("ALPeoplePerception")
        self.awareness = ALProxy("ALBasicAwareness")
        self.memory = ALProxy("ALMemory")
        self.ez = ALProxy("ALEngagementZones")

        # intialize other variables
        self.peopleInZone1 = False
        self.sonarLeftDetected = False
        self.sonarRightDetected = False
        self.obstacleDetected = False
        self.prevPeopleInZone1 = False

        # subscribe to sonar sensor
        self.sonarProxy = ALProxy("ALSonar", NAO_IP, 9559)
        self.sonarProxy.subscribe("SoundLocater")

        #to subscribe I need to give : eventname,module that will be
        #called, function that will be called
        self.memory.subscribeToEvent("ALSoundLocalization/SoundLocated",
                                "SoundLocater",
                                "onSoundLocated")
        self.memory.subscribeToEvent("PeoplePerception/PeopleDetected",
                                "SoundLocater",
                                "onPeopleDetected")
        self.memory.subscribeToEvent("EngagementZones/PeopleInZonesUpdated",
                            "SoundLocater",
                            "onPeopleInZonesUpdated")
        self.memory.subscribeToEvent("SonarLeftDetected", "SoundLocater", "onSonarLeftDetected")
        self.memory.subscribeToEvent("SonarRightDetected", "SoundLocater", "onSonarRightDetected")
        self.memory.subscribeToEvent("SonarLeftNothingDetected", "SoundLocater", "onSonarLeftNothingDetected")
        self.memory.subscribeToEvent("SonarRightNothingDetected", "SoundLocater", "onSonarRightNothingDetected")
        
        # Intialize distances
        self.ez.setSecondLimitDistance(zone2_distance)
        self.ez.setFirstLimitDistance(zone1_distance)
        self.ez.setLimitAngle(limit_angle)

        self.awareness.setEngagementMode("SemiEngaged")
        self.awareness.setTrackingMode("Head")
        self.awareness.startAwareness()

        name = "HeadPitch"
        angles = -30.0*almath.TO_RAD
        fractionMaxSpeed = 0.1
        #self.motion.setAngles(name,angles,fractionMaxSpeed)

        # for obstacle avoidance
        self.g = GridGraph()
        self.actions = []
        self.currPos = None
        self.orientation = "north"

    def onSonarLeftDetected(self):
        #if not self.sonarLeftDetected: 
        #self.tts.say("Sonar Left")
        self.sonarLeftDetected = True
        self.obstacleDetected = self.sonarLeftDetected or self.sonarRightDetected

    def onSonarRightDetected(self):
        #if not self.sonarRightDetected: 
        #self.tts.say("Sonar Right")
        self.sonarRightDetected = True
        self.obstacleDetected = self.sonarLeftDetected or self.sonarRightDetected

    def onSonarLeftNothingDetected(self):
        #if self.sonarLeftDetected: 
        #self.tts.say("Sonar Left Nothing")
        self.sonarLeftDetected = False
        self.obstacleDetected = self.sonarLeftDetected or self.sonarRightDetected

    def onSonarRightNothingDetected(self):
        #if self.sonarRightDetected: 
        #self.tts.say("Sonar Right Nothing")
        self.sonarRightDetected = False 
        self.obstacleDetected = self.sonarLeftDetected or self.sonarRightDetected

    def onPeopleDetected(self):
        print("People Detected")

    def onPeopleInZonesUpdated(self):
        """ Sets peopleInZone1 to true if there are people in Zone 1 """

        print("Zone 1: " + str(self.memory.getData("EngagementZones/PeopleInZone1")))
        print("Zone 2: " + str(self.memory.getData("EngagementZones/PeopleInZone2")))
        print("Zone 3: " + str(self.memory.getData("EngagementZones/PeopleInZone3")))

        numPeopleInZone1 = len(self.memory.getData("EngagementZones/PeopleInZone1"))
        peopleInZone1 = False if numPeopleInZone1 == 0 else True
        if numPeopleInZone1 > 0:
            self.prevPeopleInZone1 = True

    def onSoundLocated(self, *_args):
        """ Localize the sound with azimuth and make robot go into direction """

        
        array_parameters = self.self.memory.getData("ALSoundLocalization/SoundLocated")

        azimuth = array_parameters[1][0]
        altitude = array_parameters[1][1]
                
        self.tts.say("there is a sound and the azimtuh is!" + "%.2f" % azimuth)
        self.move_to_direction(azimuth)

    def moveHeadAngle(self, angle):
        name = "HeadPitch"
        angles = angle*almath.TO_RAD
        fractionMaxSpeed = 0.1
        self.motion.setAngles(name,angles,fractionMaxSpeed)

    def scan(self):
        """ Makes the robot look for humans """
        self.moveHeadAngle(-38.0)
        time.sleep(3)
        self.moveHeadAngle(38.0)

    def move_to_direction(self, azimuth):
        """ Moves into the direction of an angle """
        
        anglesTurn = {  "north": 0,
                        "east": -math.pi/2,
                        "south": math.pi,
                        "west": math.pi/2
        }

        relativePositions = {   "north": [(0, -1), (1, -1), (-1, -1)],
                                "east": [(1, 0), (1, 1), (1, -1)],
                                "south": [(0, 1), (1,1), (-1,1)],
                                "west": [(-1,0), (-1,1),(-1,-1)]}

        self.motion.moveTo(0,0,azimuth)
        self.Graph = GridGraph((15,15))
        self.Graph.bfs((7,7))
        self.prevPeopleInZone1 = False
        self.actions = self.Graph.movesTo((7,0))

        currPos = (7,7)
        goal = (7,0)
        i = 0
        steps = 15
        time.sleep(3)
        print(self.actions)
        print(self.prevPeopleInZone1)

        self.moveHeadAngle(30)

        while self.actions and not self.prevPeopleInZone1 and i < 50:
            # turn into the next direction
            action = self.actions.pop(0)
            self.motion.moveTo(0,0,anglesTurn[action] - anglesTurn[self.orientation])
            self.orientation = action
            rPos = relativePositions[action]
            if self.obstacleDetected:
                # remove obstacle from graph
                self.tts.say("obstacle")
                detection = [True, self.sonarRightDetected, self.sonarLeftDetected]
                for j in range(2):
                    if detection[j]:
                        self.g.remove(currPos[0] + rPos[j][0], currPos[0] + rPos[j][1])

                # get new path
                self.Graph.bfs(currPos)
                self.actions = self.Graph.movesTo((7,0))

                print(self.Graph)
            else:
                # make move and update position
                self.tts.say("no obstacle detected")
                self.motion.moveTo(step_size,0,0)
                # every other move
                if i % 2 == 0:
                    self.scan()
                currPos = (currPos[0] + rPos[0][0], currPos[1] + rPos[0][1])
            i += 1

        print(self.prevPeopleInZone1)

        if self.prevPeopleInZone1:
            print("hello there")
            self.tts.say("hello there")   
        if self.actions == None:
                self.tts.say("no path!")
                return
        self.Graph.setPosition(currPos)         
        print(self.Graph)


def main():
    """ Main entry point

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


    global posturemodule
    posturemodule = ALProxy("ALRobotPosture", NAO_IP, 9559)
    posturemodule.goToPosture("StandInit", 0.8) 

    #is it being declared here ? if yes I can change the name
    global SoundLocater
    SoundLocater = SoundLocaterModule("SoundLocater")

    SoundLocater.move_to_direction(0)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        SoundLocater.motion.rest()
        print("Interrupted by user, shutting down")
        myBroker.shutdown()
        sys.exit(0)



if __name__ == "__main__":
    main()
