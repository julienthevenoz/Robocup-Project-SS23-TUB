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
        
        # Intialize distances
        ez.setSecondLimitDistance(zone2_distance)
        ez.setFirstLimitDistance(zone1_distance)
        ez.setLimitAngle(limit_angle)


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
        #julien added this to look up
        # Simple command for the HeadYaw joint at 10% max speed
        names            = "HeadPitch"
        angles           = -30.0*almath.TO_RAD  #max angle is 38.5 I think
        fractionMaxSpeed = 0.1  #10% of max speed, you can change it if you want
        motionProxy.setAngles(names,angles,fractionMaxSpeed) #should be a non-blocking call
        active_looking_thread = threading.Thread(target = active_looking, daemon = True)
        active_looking_thread.start()

        
        self.move_to_direction(azimuth)

    def move_to_direction(self, azimuth):
        """ Moves into the direction of an angle """

        x = step_size * math.cos(azimuth)
        y = step_size * math.sin(azimuth)
        
        i = 0
        while not self.peopleInZone1 and i <= 5:
            walkmodule.moveTo(x,y,0)
            i += 1
        if(i < 5):
            self.tts.say("People in Zone 1 Detected. Nice to meet you!")
    def active_looking(self):
        name = "HeadYaw"
        angle1 = 45.0 * almath.TO_RAD  #look to the left
        angle2 = -45.0 * almath.TO_RAD #look to the right
        fractionMaxSpeed = 0.1 #10% of max speed, you can change it if you want
        for i in range(5):
            #turn head to the left
            motionProxy.setAngles(name, angle1, fractionMaxSpeed)
            sleep(1) #maybe you need to adjust this parameter according to the time he takes to turn his head. Maybe even delete this line ?
            #look in front
            motionProxy.setAngles(name, 0, fractionMaxSpeed)
            sleep(1)
            #look right
            motionProxy.setAngles(name, angle2, fractionMaxSpeed)
            sleep(1)
            #look in front
            motionProxy.setAngles(name, 0, fractionMaxSpeed)

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
