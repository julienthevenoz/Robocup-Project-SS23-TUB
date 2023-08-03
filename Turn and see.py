# -*- encoding: UTF-8 -*-
""" Say 'hello, you' each time a human face is detected

"""

import sys
import time
import almath

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "nao3.local"


# Global variable to store the HumanGreeter module instance
HumanGreeter = None
memory = None


class HumanGreeterModule(ALModule):
    """ A simple module able to react
    to facedetection events

    """
    
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.tts.say("hi")

        # Subscribe to the FaceDetected event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("FaceDetected",
            "HumanGreeter",
            "onFaceDetected")
        global motion
        self.motion = ALProxy("ALMotion")
        self.turning = True

    def onFaceDetected(self, *_args):
        """ This will be called each time a face is
        detected.

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        #alpha, beta, camera_pose = 0
        self.turning = False
        print("1")
        val = memory.getData("FaceDetected")
        print(val)
        sleep(1)
        timestamp = val[0]
        print("2")
        print("time", timestamp)
        alpha = val[1][0][1]
        beta =  val[1][0][2]
        print("alpha beta", alpha, " ; ", beta)
        camera_pose = val[3]  #camera pose in robot frame
        print(camera_pose)
        print("3")
        memory.unsubscribeToEvent("FaceDetected", "HumanGreeter")
        self.tts.say("Detected")
        
        sleep(2)
        # Subscribe again to the event
        memory.subscribeToEvent("FaceDetected",
            "HumanGreeter",
            "onFaceDetected")
        


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
    global HumanGreeter
    HumanGreeter = HumanGreeterModule("HumanGreeter")

    
    global walkmodule
    walkmodule = ALProxy("ALMotion", NAO_IP, 9559)
    walkmodule.setStiffnesses("Body", 1)
    walkmodule.setStiffnesses("Head", 1)
    name = "HeadPitch"
    angles = 30*almath.TO_RAD
    fractionMaxSpeed = 0.1
    walkmodule.setAngles(name,-angles,fractionMaxSpeed)
    global posturemodule
    posturemodule = ALProxy("ALRobotPosture", NAO_IP, 9559)
    posturemodule.goToPosture("StandInit", 0.5) 
    #Humangreeter.motion.moveToward(0,0,0.1)

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
