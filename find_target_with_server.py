#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math


class Server:

  def __init__(self):
    self.square_img_1 = None
    self.square_img_2 = None
    self.base_x_coord = 400
    self.base_y_coord = 550

  def img_1_callback(self, data):
    # Store message received
    self.square_img_1 = data
    # self.compute_dist_from_base()
    # self.compute_coords()

  def img_2_callback(self, data):
    # Store message received
    self.square_img_2 = data
    self.compute_dist_from_base()
    self.compute_coords()

  def compute_dist_from_base(self):
    if self.square_img_1 is not None and self.square_img_2 is not None:
      x_img_1 = self.square_img_1[0]
      x_img_2 = self.square_img_2[0]
      y_img_1 = self.square_img_1[1]
      y_img_2 = self.square_img_2[1]

      img_1_horizontal = abs(x_img_1 - self.base_x_coord)
      img_2_horizontal = abs(x_img_2 - self.base_x_coord)
      img_1_vertical = abs(y_img_1 - self.base_y_coord)
      img_2_vertical = abs(y_img_2 - self.base_y_coord)

      horizontal_distance = int(math.sqrt(img_1_horizontal^2 + img_2_horizontal^2))
      vertical_distance = int(math.sqrt(img_1_vertical^2 + img_2_vertical^2))

      distance_to_base = math.sqrt(horizontal_distance^2 + vertical_distance^2)

      print("Distance_to_base in pixels (Need to add conversion!!)")
      print(distance_to_base)

      
  def compute_coords(self):
    if self.square_img_1 is not None and self.square_img_2 is not None:
      x_img_1 = self.square_img_1[0]
      x_img_2 = self.square_img_2[0]
      y_img_1 = self.square_img_1[1]
      y_img_2 = self.square_img_2[1]

      coord_x = x_img_2
      coord_y = x_img_1
      coord_z = y_img_1

      print("Coordinates of the square")
      print("x : {}".format(coord_x))
      print("y : {}".format(coord_y))
      print("z : {}".format(coord_z))



class find_target:

  # Defines publisher and subscriber
  def __init__(self):
    
    self.server = Server()

    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    # self.image_sub1 = rospy.Subscriber("image_topic1",Image, queue_size = 1)
    # self.image_sub2 = rospy.Subscriber("image_topic1",Image, queue_size = 1)

    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,server.img_1_callback)
    # self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,server.img_2_callback)


    # print(self.image_sub1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


# Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    square_image1 = self.detect_square(cv_image1)
    # print("square_image1")
    # print(square_image1)
    self.server.img_1_callback(square_image1)

  # Recieve data from camera 1, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # print("publishing")
      # print(self.bridge.imgmsg_to_cv2(data, "bgr8"))
    except CvBridgeError as e:
      print(e)

    square_image2 = self.detect_square(cv_image2)
    # print("square_image2")
    # print(square_image2)
    self.server.img_2_callback(square_image2)


  def detect_square(self, image):
    
    mask = cv2.inRange(image, (0, 50, 50), (50, 155, 255))
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    # print(len(cnts))
    cnts = cnts[0]
    # print(cnts)
    for c in cnts:
      M = cv2.moments(c)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      shape = self.detect_shape(c)
    

      if shape == "Square":
        # print("shape")
        # print(shape)
        return np.array([cx, cy])

    # return np.array([cx, cy])


  def detect_shape(self, c):
    shape = "Unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04*peri, True)
    if len(approx) == 4:
        shape = "Square"
    else:
        shape = "Circle"
    return shape


    

# call the class
def main(args):
  
  ic = find_target()
  try:
    rospy.spin()
    # find_target()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if _name_ == '_main_':
    main(sys.argv)
