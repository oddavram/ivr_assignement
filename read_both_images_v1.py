#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from math import pi

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    self.bridge = CvBridge()
    ### Read Raw Image 1 and 2
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    
    ##### initiate topics ####
    self.Frame_pub = rospy.Publisher("Frame_Position", Float64MultiArray, queue_size=10)
    self.target_image = rospy.Publisher("Target_Position_Image", Float64MultiArray, queue_size=10)

 
  def detect_yellow(self,image):
    ## Get Frame position with respect to the image ###
    ## apply yellow mask
    lowerBound_Y = (0, 100, 100);
    upperBound_Y = (0, 255, 255);
    mask_Y = cv2.inRange(image, lowerBound_Y, upperBound_Y);
    kernel = np.ones((5, 5), np.uint8)
    mask_Y = cv2.dilate(mask_Y, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_Y, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_Y = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return (centroid_Y)

  def detect_blue(self,image):
    ## Get Frame position with respect to the image ###
    ## apply yellow mask
    lowerBound_B = (100, 0, 0);
    upperBound_B = (255, 5, 5);
    mask_B = cv2.inRange(image, lowerBound_B, upperBound_B);
    kernel = np.ones((5, 5), np.uint8)
    mask_B = cv2.dilate(mask_B, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_B, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_B = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return (centroid_B)

  def detect_green(self,image):
    lowerBound_G = (0, 100, 0);
    upperBound_G = (5, 255, 5);
    mask_G = cv2.inRange(image, lowerBound_G, upperBound_G);
    kernel = np.ones((5, 5), np.uint8)
    mask_G = cv2.dilate(mask_G, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_G, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_G = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return centroid_G

  def detect_red(self,image):
    lowerBound_R = (0, 0, 100);
    upperBound_R = (5, 5, 255);
    mask_R = cv2.inRange(image, lowerBound_R, upperBound_R);
    kernel = np.ones((5, 5), np.uint8)
    mask_R = cv2.dilate(mask_R, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_R, 1, 2)
    #if len(contours) == 0:
    #  return 0
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_R = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return centroid_R

  def detect_orange(self,image):
    lowerBound_O = (5, 52, 102);
    upperBound_O = (40, 162, 250);
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_O = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
    # kernel = np.ones((5, 5), np.uint8)
    # mask_O = cv2.dilate(mask_O, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_O, 1, 2)
    # Plot image for verification
    return contours

  def get_px_meters(self,pos_center,pos_2nd_joint):
    link_length = 2.5
    ## joint 1 only rotates around z!!
    px_difference = abs(pos_center[2] - pos_2nd_joint[2])
    return link_length/px_difference

  def calc_pos_global_pxlsAndMeters(self, px_to_meters, pos_center, pos):
    pos_global = (pos - pos_center)
    ## Z axis is inverted ##
    pos_global[2] = - pos_global[2]
    pos_global_meters = pos_global * px_to_meters
    return pos_global, pos_global_meters
  
  
  def get_object_features(self,contours):
    aspect_ratios = []
    rectness = []
    circless = []
    for contour in contours:
      contour_area = cv2.contourArea(contour)
      x, y, w, h = cv2.boundingRect(contour)
      min_rect_area = cv2.minAreaRect(contour)
      min_rect_area = min_rect_area[1][0] * min_rect_area[1][1]
      (x, y), radius = cv2.minEnclosingCircle(contour)
      radius = int(radius)
      aspect_ratios.append(w / h)
      rectness.append(contour_area / min_rect_area)
      circless.append(contour_area / (pi * radius ** 2))
    return (aspect_ratios, rectness, circless)

  def recognize_sphere(self,aspect_ratios, rectness, circless):
    sphericity = [aspect_ratios[idx] - rectness[idx] + circless[idx] for idx in range(len(aspect_ratios))]
    if max(sphericity) < 0.6:
      print('No sphere was detected! Sphere behind Robot!')
      return -1
    else:
      return sphericity.index(max(sphericity))

  def get_pos_sphere(self, contour, sphere_contour_idx):
      cnt = contour[sphere_contour_idx]
      M = cv2.moments(cnt)
      centroid_sphere = [M['m10'] / M['m00'], M['m01'] / M['m00']]
      return centroid_sphere

  def detect_sphere_pos(self,image):
    contours = self.detect_orange(image)
    aspect_ratios, rectness, circless = self.get_object_features(contours)
    sphere_contour_idx = self.recognize_sphere(aspect_ratios, rectness, circless)
    if sphere_contour_idx == -1:
      return (0, -1)
    return self.get_pos_sphere(contours, sphere_contour_idx)
 

  # Recieve data from camera 1, process it, and publish
  def callback(self,image1, image2):
    # Recieve the image
    try:
      image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      #im = cv2.imshow("Image1", image1)
      #cv2.waitKey(1)
      image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
      #im2 = cv2.imshow("Image2", image2)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)
######################### My Code ###########################
    y_center, z_center_1 = self.detect_yellow(image1)
    x_center, z_center2 = self.detect_yellow(image2)
    z_center = 0.5*(z_center_1+z_center2)
    pos_center = np.array([x_center, y_center, z_center])
    self.pos_center = Float64MultiArray()
    self.pos_center.data = pos_center
    self.Frame_pub.publish(self.pos_center)
    
    
    y_2nd_joint, z_2nd_joint_1 = self.detect_blue(image1)
    x_2nd_joint, z_2nd_joint_2 = self.detect_blue(image2)
    z_2nd_joint = 0.5*(z_2nd_joint_1+z_2nd_joint_2)
    pos_2nd_joint = np.array([x_2nd_joint, y_2nd_joint, z_2nd_joint])
    #print(pos_2nd_joint)
    
    px_to_meters = self.get_px_meters(pos_center, pos_2nd_joint)
    
    #print(px_to_meters)
    y,z1 = self.detect_sphere_pos(image1)
    x,z2 = self.detect_sphere_pos(image2)
    if z1 == -1:
       z_final = z2
    elif z2 == -1:
       z_final = z1
    else:
       z_final = 0.5*(z1+z2)
       
    pos_sphere = np.array([x,y,z_final])
    pos_sphere_pxl_global, pos_sphere_meter_global =        self.calc_pos_global_pxlsAndMeters(px_to_meters, pos_center, pos_sphere)
    distance_to_center = np.sqrt(np.sum([i**2 for i in pos_sphere_meter_global]))
    self.pos_sphere = Float64MultiArray()
    self.pos_sphere.data = pos_sphere_meter_global
    self.target_image.publish(self.pos_sphere)   
    
    
###############################################################


# call the class
def main(args):
  img_cvt = image_converter()
  rospy.init_node('image_processing', anonymous=True)
  try:
    ts = message_filters.ApproximateTimeSynchronizer([img_cvt.image_sub1, img_cvt.image_sub2], 10, 0.1)
    ts.registerCallback(img_cvt.callback)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


