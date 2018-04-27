#!/usr/bin/env python
import signal
import interpolate
import atexit
import time
import rospy
from truck_drive import Truck
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

class CommandNode:
    def __init__(self):
        
        interpolate.generateDictionaries()
        interpolate.setRosParams()

        self.last_message_time = 0
        self.last_speed = 0


        #Boolean result from the distance measurement 
        self.allowedToDrive = True
        self.rogue_det_ok   = True
        self.std_det_ok     = True
        #self.dist_det_ok    = True
        

        
        self.truck = Truck()
        self.truck.reset()
        self.truck.update()
        
        
        rospy.init_node('cmd_node', anonymous=True)
        rospy.Subscriber('master_drive', AckermannDrive, self.callback)
        #Subscribe to distance detector
        rospy.Subscriber('chatter', String, self.callback_ts) 
        rospy.Subscriber('rogue_detector', String, self.callback_rogue_det)
        rospy.Subscriber('std_detector', String, self.callback_std_det)                        
        #Callback function for the distance detector topic 
    def callback_ts(self, data):
	if data.data == "STOP":
            self.allowedToDrive = False
	if data.data == "START": 
            self.allowedToDrive = True

    def callback_rogue_det(self, data):
	if data.data == "STOP":
            self.rogue_det_ok = False
	if data.data == "START": 
	    self.rogue_det_ok = True

    def callback_std_det(self, data):
	if data.data == "STOP":
            self.std_det_ok = False
	if data.data == "START": 
	    self.std_det_ok = True
            
    
    def callback(self, data):
        #Take steering insructions into account only
        #when we are allowed to drive
        if (self.allowedToDrive and self.rogue_det_ok and self.std_det_ok):
            phi = data.steering_angle
            v = data.speed

            self.last_message_time = rospy.get_time()
            self.last_speed = v

            # look up what command to send to truck
            steering_cmd = interpolate.getSteeringCmd(phi)
            speed_cmd = interpolate.getSpeedCmd(v)

            self.truck.setSteering(steering_cmd)
            self.truck.setSpeed(speed_cmd)
            self.truck.update()
	else:
	    speed_cmd = interpolate.getSpeedCmd(0)
            self.truck.setSpeed(speed_cmd)
            self.truck.update() 
	    

    def spin(self): 
        #rospy.spin()    
        while not rospy.is_shutdown():
            #if truck is moving and last message was a long time ago, stop truck
            if abs(self.last_speed) >= 0:
                lastmsg = rospy.get_time() - self.last_message_time
                if lastmsg > 0.2:
                    rospy.logwarn("cmd_node: didnt receive a message in 0.2 sec, resetting. last message = %s", lastmsg)
                    self.truck.reset()
                    self.truck.update()
            
            rospy.sleep(0.1)

# on ctrl c
def interruptHandler(sig, frame):
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	rospy.loginfo("Interrupted, reset servo...")
	# Return to neutral
	cn = CommandNode()
	cn.truck.reset()
	cn.truck.update()
	time.sleep(0.4)
	exit(0)

#if program crashes
def exit_handler():
    rospy.logfatal("exit handler in cmd_node invoked")
    cn = CommandNode()
    cn.truck.reset()
    cn.truck.update()
    exit(0)

signal.signal(signal.SIGINT, interruptHandler)
atexit.register(exit_handler)

if __name__ == '__main__':
    cn = CommandNode()
    cn.spin()



