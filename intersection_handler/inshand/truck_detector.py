import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String


#red color HSV-limits for detecting trucks
upper_red = np.array([255, 255, 255])
lower_red = np.array([150, 100, 75])

params_grill = cv2.SimpleBlobDetector_Params()
params_grill.filterByColor = True
params_grill.blobColor = 255

params_grill.filterByInertia = True
params_grill.minInertiaRatio = 0
params_grill.maxInertiaRatio = 1

params_grill.filterByConvexity = True
params_grill.minConvexity = 0.60


detector_grill = cv2.SimpleBlobDetector(params_grill)

def filter_red_one(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    filtered_frame_red = cv2.bitwise_and(frame, frame, mask=mask_red)
    
    return(mask_red, filtered_frame_red)

def filter_red_two(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    filtered_frame_red = cv2.bitwise_and(frame, frame, mask=mask_red)
    
    return(mask_red, filtered_frame_red)

def apply_filter_1_and_2(image):
    mask_0, img_0 = filter_red_one(image)
    mask_1, img_1 = filter_red_two(image)
    filtered_frame = cv2.bitwise_or(img_0, img_1)
    combined_mask = cv2.bitwise_or(mask_0, mask_1)
    return combined_mask, filtered_frame


def filter_objects_grill(binarized_frame):
    keypoints_grill = detector_grill.detect(binarized_frame)
    img_with_keypoints_grill = cv2.drawKeypoints(binarized_frame, keypoints_grill, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return(keypoints_grill, img_with_keypoints_grill)


class TruckDetect:
    def __init__(self):
        self.std_pub = rospy.Publisher('std_detector', String, queue_size=10)
        self.rogue_pub = rospy.Publisher('rogue_detector', String, queue_size=10)
        self.look_for_std = False
        rospy.init_node('truck_detector', anonymous=True)
        rate = rospy.Rate(10)

        rospy.Subscriber('section_lock', String, self.callback_lock)

    def callback_lock(self, data):
        if data.data == "zk_connection_failed":
            #print("zk is down")
            self.look_for_std = True
                
        
    def truck_stop_decision(self):
        cap = cv2.VideoCapture(0)

        if (self.look_for_std):
            while("pigs don't fly"):
            	ret, frame = cap.read()
            	mask_red, filtered_frame_red = apply_filter_1_and_2(frame)
            	keypoints_grill_red, img_with_keypoints_grill_red = filter_objects_grill(mask_red)
            	ts = 0
		if len(keypoints_grill_red) > 2:
                    self.std_pub.publish("STOP")
                    ts = time.time()
            	else:
                    ts2 = time.time()
                    if ts2 > ts+7:
                        self.std_pub.publish("START")
                

        while("pigs don't fly"):
            ret, frame = cap.read()
            mask_red, filtered_frame_red = apply_filter_1_and_2(frame)
            keypoints_grill_red, img_with_keypoints_grill_red = filter_objects_grill(mask_red)
            ts = 0 
	    if len(keypoints_grill_red) > 2:
                self.rogue_pub.publish("STOP")
                ts = time.time()
            else:
                ts2 = time.time()
                if ts2 > ts+7:
                    self.rogue_pub.publish("START")

        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        ltd = TruckDetect()
        #rospy.spin()
        ltd.truck_stop_decision()
    except rospy.ROSInterruptException:
        pass
