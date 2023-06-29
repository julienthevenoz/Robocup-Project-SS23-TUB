# -*- encoding: UTF-8 -*-
""" Say 'hello, you' each time a human face is detected

"""

import sys
import time

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "10.0.7.100"


# Global variable to store the HumanGreeter module instance
HumanGreeter = None
memory = None

#not clear : the class is called SoundLocaterModule
#but the module is called SoundLocater (name passed to it) ?
class SoundLocaterModule(ALModule):   
    """ A simple module able to react
    to facedetection events

    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.tts.say("start")

        # Subscribe to the FaceDetected event:
        global memory
        memory = ALProxy("ALMemory")
        #to subscribe I need to give : eventname,module that will be
        #called, function that will be called
        memory.subscribeToEvent("ALSoundLocalization/SoundLocated",
                                "SoundLocater",
                                "onSoundLocated")

        #initialing memory variables for frequency
        memory.insertData("nbSoundsHeard", 0)
        memory.insertData("hearingStartTime", 0)
        memory.insertData("hearingLastTime", 0)

    #i changed this but is it correct ?
    def onSoundLocated(self, *_args):
        """ This will be called each time a face is
        detected.

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions

        #to unsubscribe we only need eventname and module
        memory.unsubscribeToEvent("ALSoundLocalization/SoundLocated",
                                  "SoundLocater")
        self.tts.say("Sound detected")

        #we heard one more sound
        nbSoundsHeard = memory.getData("nbSoundsHeard")
        nbSoundsHeard += 1
        memory.insertData("nbSoundsHeard", nbSoundsHeard)
        self.tts.say(str(nbSoundsHeard))

        #now let's try to deal with sound
        heardTimeALValue = memory.getTimestamp("nbSoundsHeard") #this returns An ALValue with 3 items: * memoryKey value, * first part of the timestamp (seconds), * second part of the timestamp (microseconds).
        print("heardTimeALValue", heardTimeALValue)
        sys.stdout.flush()  #on flush ouais
        heardTime = heardtimeALValue[1] + heardTimeALValue[2]*0.000001
        print("heardTime",heardTime)
        sys.stdout.flush() #on flush ouais

        hearingStartTime = memory.getData("hearingStartTime")
        
        if(hearingStartTime == 0):
            memory.insertData("hearingStartTime", heardTime)
        else:
            freq = nbSoundsHeard /(heardTime - hearingStartTime)
           # print("freq : ", freq)
           # sys.stdout.flush()#on flush ouais 
        print()
        print() #two white lines just for clarity
            

        
       # walkmodule.moveInit()
       # walkmodule.moveTo(0,0.1,1)
       # walkmodule.setStiffnesses("Body", 1)  #redundnat ?

        # Subscribe again to the event
        memory.subscribeToEvent("ALSoundLocalization/SoundLocated",
                                "SoundLocater",
                                "onSoundLocated")


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


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable

   

    #test to make him turn on himself when he hears a noise
    global walkmodule
    walkmodule = ALProxy("ALMotion", NAO_IP, 9559)
    walkmodule.setStiffnesses("Body", 1)
    #motionProxy.wakeUp() useful ?



    global posturemodule
    posturemodule = ALProxy("ALRobotPosture", NAO_IP, 9559)
    posturemodule.goToPosture("StandInit", 0.5) 

    #is it being declared here ? if yes I can change the name
    global SoundLocater
    SoundLocater = SoundLocaterModule("SoundLocater")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        walkmodule.rest()
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)



if __name__ == "__main__":
    main()

