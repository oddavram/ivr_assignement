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
    self.joint_2_img_1 = None
    self.joint_3_img_1 = None
    self.joint_4_img_1 = None
    self.joint_2_img_2 = None
    self.joint_3_img_2 = None
    self.joint_4_img_2 = None
    self.pixel2meter = None

  def joint_img_1_callback(self, joint2, joint3, joint4):
    # Store message received
    self.joint_2_img_1 = joint2
    self.joint_3_img_1 = joint3
    self.joint_4_img_1 = joint4

    # self.compute_dist_from_base()
    # self.compute_coords()

  def joint_img_2_callback(self, joint2, joint3, joint4):
    # Store message received
    self.joint_2_img_2 = joint2
    self.joint_3_img_2 = joint3
    self.joint_4_img_2 = joint4
    self.compute_angle_joint2()
    self.compute_angle_joint3()
    self.compute_angle_joint4()

  def pixel2meter_callback(self, data):
    self.pixel2meter = data

    # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,image):
    a = self.pixel2meter
    print("a")
  #   # Obtain the centre of each coloured blob 
  #   center = a * self.detect_yellow(image)
  #   circle1Pos = a * self.detect_blue(image) 
  #   circle2Pos = a * self.detect_green(image) 
  #   circle3Pos = a * self.detect_red(image)
  #   # Solve using trigonometry
  #   ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
  #   ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
  #   ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
  #   return np.array([ja1, ja2, ja3])
  # cv_images


  def compute_angle_joint2(self):
    # if self.square_img_1 is not None and self.square_img_2 is not None:
    print("testj2")
    print("self.joint_2_img_1")
    print(self.joint_2_img_1)
    print("self.joint_2_img_2")
    print(self.joint_2_img_2)
      # x_img_1 = self.square_img_1[0]
      # x_img_2 = self.square_img_2[0]
      # y_img_1 = self.square_img_1[1]
      # y_img_2 = self.square_img_2[1]

      # img_1_horizontal = abs(x_img_1 - self.base_x_coord)
      # img_2_horizontal = abs(x_img_2 - self.base_x_coord)
      # img_1_vertical = abs(y_img_1 - self.base_y_coord)
      # img_2_vertical = abs(y_img_2 - self.base_y_coord)

      # horizontal_distance = int(math.sqrt(img_1_horizontal^2 + img_2_horizontal^2))
      # vertical_distance = int(math.sqrt(img_1_vertical^2 + img_2_vertical^2))

      # distance_to_base = math.sqrt(horizontal_distance^2 + vertical_distance^2)

      # print("Distance_to_base in pixels (Need to add conversion!!)")
      # print(distance_to_base)

      
  def compute_angle_joint3(self):
    # if self.square_img_1 is not None and self.square_img_2 is not None:
    print("testj3")
    print("self.joint_3_img_1")
    print(self.joint_3_img_1)
    print("self.joint_3_img_2")
    print(self.joint_3_img_2)
      # x_img_1 = self.square_img_1[0]
      # x_img_2 = self.square_img_2[0]
      # y_img_1 = self.square_img_1[1]
      # y_img_2 = self.square_img_2[1]

      # coord_x = x_img_2
      # coord_y = x_img_1
      # coord_z = y_img_1

      # print("Coordinates of the square")
      # print("x : {}".format(coord_x))
      # print("y : {}".format(coord_y))
      # print("z : {}".format(coord_z))

  def compute_angle_joint4(self):
    # if self.square_img_1 is not None and self.square_img_2 is not None:
    print("testj4")
    print("self.joint_4_img_1")
    print(self.joint_4_img_1)
    print("self.joint_4_img_2")
    print(self.joint_4_img_2)



class find_angles:

  # Defines publisher and subscriber
  def __init__(self):
    
    self.server = Server()

    self.first_attempt = None

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


  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
  

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    if self.first_attempt == None:
      self.first_attempt = 1
      # Calculate the conversion from pixel to meter
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(cv_image1)
      circle2Pos = self.detect_green(cv_image1)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      self.server.pixel2meter_callback(3 / np.sqrt(dist))

    center = self.detect_yellow(cv_image1)
    circle1Pos = self.detect_blue(cv_image1) 
    circle2Pos = self.detect_green(cv_image1) 
    circle3Pos = self.detect_red(cv_image1)

    self.server.joint_img_1_callback(circle1Pos, circle2Pos, circle3Pos)

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

    center = self.detect_yellow(cv_image2)
    circle1Pos = self.detect_blue(cv_image2) 
    circle2Pos = self.detect_green(cv_image2) 
    circle3Pos = self.detect_red(cv_image2)

    self.server.joint_img_2_callback(circle1Pos, circle2Pos, circle3Pos)



# call the class
def main(args):
    
  ic = find_angles()
  try:
    rospy.spin()
    # find_target()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if _name_ == '_main_':
    main(sys.argv)
