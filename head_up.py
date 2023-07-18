import time
import almath
import argparse
from naoqi import ALProxy
import threading

def main(robotIP, PORT=9559):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    motionProxy.setStiffnesses("Body", 1.0)  #need stifness to power the joints

    # Simple command for the HeadYaw joint at 10% max speed
    names            = "HeadPitch"
    angles           = -30.0*almath.TO_RAD  #max angle is 38.5 I think
    fractionMaxSpeed = 0.1  #10% of max speed, you can change it if you want
    motionProxy.setAngles(names,angles,fractionMaxSpeed) #should be a non-blocking call

    #this is a thread to make him look left and right. If it doesn't work just throw it away.
    left_to_right_thread = threading.Thread(target = left_to_right, daemon = True)
    left_to_right_thread.start()

    #let's make him walk forward
    motionProxy.moveTo(1, 0, 0)
    
    time.sleep(3.0)


def left_to_right():
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
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
