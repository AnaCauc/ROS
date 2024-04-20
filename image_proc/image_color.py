#!/usr/bin/env python3 
import numpy as np
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2 

bridge_object = CvBridge() # create the cv_bridge object 
image_received = 0 #Flag to indicate that we have already received an image 
cv_image = 0 #This is just to create the global variable cv_image 

def show_image(): 
    image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,camera_callback) 
    r = rospy.Rate(10) #10Hz 
    while not rospy.is_shutdown(): 
        if image_received: 
            cv2.waitKey(1) 
            r.sleep() 
    cv2.destroyAllWindows() 

def process_image(image):
    image = cv2.resize(image,(300,300)) 
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

    min_red = np.array([0,70,50]) 
    max_red = np.array([170,255,255]) 
    mask_r = cv2.inRange(hsv, min_red, max_red) 

    res_r = cv2.bitwise_and(image,image, mask= mask_r) 
    cv2.imshow('Red',res_r) 
    cv2.imshow('Original',image) 
    cv2.waitKey(1) 
    
def camera_callback(data): 
    global bridge_object 
    global cv_image 
    global image_received 
    image_received=1 
    try: 
        print("received ROS image, I will convert it to opencv") 
        # We select bgr8 because its the OpenCV encoding by default 
        cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        #Add your code to save the image here: 
        #Save the image "img" in the current path 
        cv2.imwrite('robot_image.jpg', cv_image) 
        ## Calling the processing function
        process_image(cv_image)
        cv2.imshow('Image from robot camera', cv_image) 
    except CvBridgeError as e: 
        print(e) 

if __name__ == '__main__': 
    rospy.init_node('load_image', anonymous=True) 
    show_image() 