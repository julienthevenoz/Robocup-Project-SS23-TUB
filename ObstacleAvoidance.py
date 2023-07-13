# -*- encoding: UTF-8 -*-
""" Say 'hello, you' each time a human face is detected

"""

import sys
import time

import math


from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "10.0.7.113"

zone1_distance = 2.0
zone2_distance = 3.0
limit_angle = 90.0

step_size = 0.25

#not clear : the class is called SoundLocaterModule
#but the module is called SoundLocater (name passed to it) ?
class SoundLocaterModule(ALModule):   
    """ A simple module able to react
    to facedetection events

    """
    def __init__(self, name):
        ALModule.__init__(self, name)
    
        # instantiate all necessary proxys
        self.tts = ALProxy("ALTextToSpeech")
        self.motion = ALProxy("ALMotion")
        self.peoplePerception = ALProxy("ALPeoplePerception")
        self.memory = ALProxy("Memory")
        self.ez = ALProxy("ALEngagementZones")

        # intialize other variables
        self.peopleInZone1 = False
        self.sonarLeftDetected = False
        self.sonarRightDetected = False

        # subscribe to sonar sensor
        ALSonarProxy::subscribe
        self.sonarProxy = ALProxy("ALSonarProxy")
        sonarProxy.subscribe("SoundLocater")

        #to subscribe I need to give : eventname,module that will be
        #called, function that will be called
        memory.subscribeToEvent("ALSoundLocalization/SoundLocated",
                                "SoundLocater",
                                "onSoundLocated")
        memory.subscribeToEvent("PeoplePerception/PeopleDetected",
                                "SoundLocater",
                                "onPeopleDetected")
        memory.subscribeToEvent("EngagementZones/PeopleInZonesUpdated",
                            "SoundLocater",
                            "onPeopleInZonesUpdated")
        memory.subscribeToEvent("SonarLeftDetected", "SoundLocater", "onSonarLeftDetected")
        memory.subscribeToEvent("SonarRightDetected", "SoundLocater", "onSonarRightDetected")
        memory.subscribeToEvent("SonarLeftNothingDetected", "SoundLocater", "onSonarLeftNothingDetected")
        memory.subscribeToEvent("SonarRightDetected", "SoundLocater", "onSonarRightNothingDetected")
        
        # Intialize distances
        ez.setSecondLimitDistance(zone2_distance)
        ez.setFirstLimitDistance(zone1_distance)
        ez.setLimitAngle(limit_angle)

    def onSonarLeftDetected(self):
        self.sonarLeftDetected = True

    def onSonarRightDetected(self):
        self.sonarRightDetected = True

    def onSonarLeftNothingDetected(self):
        self.sonarLeftDetected = False

    def onSonarRightNothingDetected(self):
        self.sonarRightDetected = False 

    def onPeopleDetected(self):
        print("People Detected")

    def onPeopleInZonesUpdated(self):
        """ Sets peopleInZone1 to true if there are people in Zone 1 """

        print("Zone 1: " + str(memory.getData("EngagementZones/PeopleInZone1")))
        print("Zone 2: " + str(memory.getData("EngagementZones/PeopleInZone2")))
        print("Zone 3: " + str(memory.getData("EngagementZones/PeopleInZone3")))

        numPeopleInZone1 = len(memory.getData("EngagementZones/PeopleInZone1"))
        peopleInZone1 = False if numPeopleInZone1 == 0 else True

    def onSoundLocated(self, *_args):
        """ Localize the sound with azimuth and make robot go into direction """

        
        array_parameters = memory.getData("ALSoundLocalization/SoundLocated")

        azimuth = array_parameters[1][0]
        altitude = array_parameters[1][1]
                
        self.tts.say("there is a sound and the azimtuh is!" + "%.2f" % azimuth)
        self.move_to_direction(azimuth)

    def move_to_direction(self, azimuth):
        """ Moves into the direction of an angle """

        x = step_size * math.cos(azimuth)
        y = step_size * math.sin(azimuth)
        
        i = 0
        while not self.peopleInZone1 and i <= 5:
            if self.sonarLeftDetected or self.sonarRightDetected:
                obstacleAvoidance(azimuth)
            walkmodule.moveTo(x,y,0)
            i += 1
        if(i < 5):
            self.tts.say("People in Zone 1 Detected. Nice to meet you!")

    def obstacleAvoidance(self, azimuth):
        
        maneuvers = {(True, False): obstacleAvoidanceRight(azimuth),
                     (False, True): obstacleAvoidanceLeft(azimuth),
                     (True, True): largeObstacleAvoidance(azimuth),
                     (False, False): None }

        sonars = (sonarLeftDetected, sonarRightDetected)
        return maneuvers[sonars]

    def obstacleAvoidanceLeft(self, azimuth):
        """ If theres an obstacle to the, go around it on the left """
        x = step_size * math.cos(azimuth + (math.pi/4))
        y = step_size * math.sin(azimuth + (math.pi/4))
        self.motion.moveTo(x,y)

        x = step_size * math.cos(azimuth - (math.pi/2))
        y = step_size * math.sin(azimuth - (math.pi/2))
        self.motion.moveTo(x,y)

    def obstacleAvoidanceRight(self, azimuth):
        """ If theres an obstacle to the, go around it on the left """
        x = step_size * math.cos(azimuth - (math.pi/4))
        y = step_size * math.sin(azimuth - (math.pi/4))
        self.motion.moveTo(x,y)

        x = step_size * math.cos(azimuth + (math.pi/2))
        y = step_size * math.sin(azimuth + (math.pi/2))
        self.motion.moveTo(x,y)

    def largeObstacleAvoidance(self, azimuth):
        """ If theres an obstacle to the, go around it on the left """
        x = step_size * math.cos(azimuth - (math.pi/2))
        y = step_size * math.sin(azimuth - (math.pi/2))
        self.motion.moveTo(x,y)

        x = step_size * math.cos(azimuth + (math.pi/2))
        y = step_size * math.sin(azimuth + (math.pi/2))
        self.motion.moveTo(x,y)

        x = step_size * math.cos(azimuth + (math.pi/2))
        y = step_size * math.sin(azimuth + (math.pi/2))
        self.motion.moveTo(x,y)

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

    SoundLocater.move_()
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
