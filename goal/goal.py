#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2
import numpy as np


#funcția de găsire a coordonatelor unui punct mai puțin vizibil
def find_first_white_pixel(image_path, p1, p2):
	#se citește harta salvată și se extrag dimensiunile imaginii
    image = cv2.imread(image_path)
    height, width, channels = image.shape
	
	#se caută punctul mai puțin vizibil
    for p in np.linspace(p1, p2, np.linalg.norm(p1 - p2)):
        pos = tuple(np.int32(p))
        if (image[pos[0], pos[1], 0] == 254 and
            image[pos[0], pos[1], 1] == 254 and
            image[pos[0], pos[1], 2] == 254):
            return pos

    return None

#funcția de deplasare a robotului în punctul dorit
def movebase_client(x, y, orientation_z, orientation_w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = orientation_z
    goal.target_pose.pose.orientation.w = orientation_w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


#transformarea coordonatelor din pixeli în coordonate ROS
def convert_to_ros_coordinates(pixel_coordinates, image_width, image_height, 	ros_environment_width, ros_environment_height):

# calcularea factorilor(valori?) de scalare  
    x_scale = ros_environment_width / image_width
    y_scale = ros_environment_height / image_height

    #calcularea valorilor de offset, ținând cont că originea hărții în pixeli diferă de originea hărții din Gazebo
    x_offset = -ros_environment_width / 2.0
    y_offset = -ros_environment_height / 2.0

    # Transformarea coordonatelor din pixeli în coordonate ROS
    ros_x = pixel_coordinates[1] * x_scale + x_offset
    ros_y = -pixel_coordinates[0] * y_scale + y_offset

    return ros_x, ros_y


#inițializarea nodului și valorilor necesare pentru locația imaginii și coordonatele inițiale ale robotului
rospy.init_node('goal_node')
image_path = 'Downloads/docs-main/docs-main/line_of_sight/result.png'
pt_a = np.array([0, 0])
pt_b = np.array([220, 210])

#găsirea primului punct mai puțin vizibil
first_white_pixel = find_first_white_pixel(image_path, pt_a, pt_b)

#dacă am găsit punctul, se vor transforma coordonatele din pixeli în coordonate ROS și se va deplasa robotul în acea locație
if first_white_pixel is not None:
	#se inițializează dimensiunile mediului ros
    ros_environment_width= (192-first_white_pixel[1])*0.05
    ros_environment_height=(192-first_white_pixel[0])*0.05
    #se transformă coordonatele
    ros_coordinates = convert_to_ros_coordinates(first_white_pixel, 384, 384, 	ros_environment_width, ros_environment_height)
    print("ROS Coordinates of the first white pixel:", ros_coordinates)

    # se deplasează robotul în direcția dorită
    result=movebase_client(ros_coordinates[0], ros_coordinates[1], 0.0, 1.0)
else:
    print("No white pixel found in the specified region.")
if result:
    rospy.loginfo("Goal execution done!")