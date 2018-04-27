import rospy
import rospkg
from std_msgs.msg import *
#from custom_msgs.msg import *

class zkDownTest:
    def __init__(self):
        rospy.init_node('zk_down_test', anonymous=True)
        self.pub = rospy.Publisher('section_lock', String, queue_size=0)

    def signal_zk_down(self):
        self.pub.publish("zk_connection_failed")
if __name__ == '__main__':
    s = zkDownTest()
    i = 0
    while(i < 100000):
	i = i+1
        s.signal_zk_down()
    #rospy.spin()    

