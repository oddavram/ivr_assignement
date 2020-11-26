#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from math import pi
from math import sin, cos

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    self.bridge = CvBridge()
    ### Read Raw Image 1 and 2
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.angles = message_filters.Subscriber("/robot/joint_states",JointState)
    #self.target_image_sub = message_filters.Subscriber("Target_Position_Image",Float64MultiArray,queue_size=10)
    ##### initiate topics ####
    self.Frame_pub = rospy.Publisher("Frame_Position", Float64MultiArray, queue_size=10)
    self.target_image = rospy.Publisher("Target_Position_Image", Float64MultiArray, queue_size=10)
    self.end_pos = rospy.Publisher("End_Position", Float64MultiArray, queue_size=10)
 
    #### Topics for joint commands ###
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
 
    
    ## Time Topics ###    
    self.time_trajectory = rospy.get_time()
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')   
    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0, 0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0, 0.0], dtype='float64') 


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
  

  def F_K_full(self, theta1, theta2, theta3, theta4):
    Trans_full = np.array([[- cos(theta4) * (cos(theta1) * sin(theta3) + cos(theta3) * sin(theta1) * sin(theta2)) - cos(
        theta2) * sin(theta1) * sin(theta4),
                            sin(theta4) * (cos(theta1) * sin(theta3) + cos(theta3) * sin(theta1) * sin(theta2)) - cos(
                                theta2) * cos(theta4) * sin(theta1),
                            cos(theta1) * cos(theta3) - sin(theta1) * sin(theta2) * sin(theta3),
                            (7 * cos(theta1) * sin(theta3)) / 2 + 3 * cos(theta4) * (
                                        cos(theta1) * sin(theta3) + cos(theta3) * sin(theta1) * sin(theta2)) + (
                                        7 * cos(theta3) * sin(theta1) * sin(theta2)) / 2 + 3 * cos(theta2) * sin(
                                theta1) * sin(theta4)
                            ],
                           [cos(theta1) * cos(theta2) * sin(theta4) - cos(theta4) * (
                                       sin(theta1) * sin(theta3) - cos(theta1) * cos(theta3) * sin(theta2)),
                            sin(theta4) * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta3) * sin(theta2)) + cos(
                                theta1) * cos(theta2) * cos(theta4),
                            cos(theta3) * sin(theta1) + cos(theta1) * sin(theta2) * sin(theta3),
                            (7 * sin(theta1) * sin(theta3)) / 2 + 3 * cos(theta4) * (
                                        sin(theta1) * sin(theta3) - cos(theta1) * cos(theta3) * sin(theta2)) - (
                                        7 * cos(theta1) * cos(theta3) * sin(theta2)) / 2 - 3 * cos(theta1) * cos(
                                theta2) * sin(theta4)
                            ],
                           [sin(theta2) * sin(theta4) - cos(theta2) * cos(theta3) * cos(theta4),
                            cos(theta4) * sin(theta2) + cos(theta2) * cos(theta3) * sin(theta4),
                            -cos(theta2) * sin(theta3),
                            (7 * cos(theta2) * cos(theta3)) / 2 - 3 * sin(theta2) * sin(theta4) + 3 * cos(theta2) * cos(
                                theta3) * cos(theta4) + 5 / 2],
                           [0, 0, 0, 1]
                           ])
    return Trans_full

  def get_jacobian(self, theta1, theta2, theta3, theta4):
    jacob = [[(7*cos(theta1)*cos(theta3)*sin(theta2))/2 - 3*cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - (7*sin(theta1)*sin(theta3))/2 + 3*cos(theta1)*cos(theta2)*sin(theta4),
     (7*cos(theta2)*cos(theta3)*sin(theta1))/2 - 3*sin(theta1)*sin(theta2)*sin(theta4) + 3*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1),
     (7*cos(theta1)*cos(theta3))/2 + 3*cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - (7*sin(theta1)*sin(theta2)*sin(theta3))/2,
     3*cos(theta2)*cos(theta4)*sin(theta1) - 3*sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))],
    [(7*cos(theta1)*sin(theta3))/2 + 3*cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) + (7*cos(theta3)*sin(theta1)*sin(theta2))/2 + 3*cos(theta2)*sin(theta1)*sin(theta4),
     3*cos(theta1)*sin(theta2)*sin(theta4) - (7*cos(theta1)*cos(theta2)*cos(theta3))/2 - 3*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4),
     (7*cos(theta3)*sin(theta1))/2 + 3*cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) + (7*cos(theta1)*sin(theta2)*sin(theta3))/2,
     - 3*sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - 3*cos(theta1)*cos(theta2)*cos(theta4)],
    [0,
     - (7*cos(theta3)*sin(theta2))/2 - 3*cos(theta2)*sin(theta4) - 3*cos(theta3)*cos(theta4)*sin(theta2),
     - (7*cos(theta2)*sin(theta3))/2 - 3*cos(theta2)*cos(theta4)*sin(theta3),
     - 3*cos(theta4)*sin(theta2) - 3*cos(theta2)*cos(theta3)*sin(theta4)]]
    return jacob

 
      
  def control_closed(self,pos_d, q):#,image):
    # P gain
    K_p = np.array([[4,0,0],[0,4,0], [0,0,4]])
    # D gain
    K_d = np.array([[0.04,0,0],[0,0.04,0],[0,0.04,0]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    F_K = self.F_K_full(q[0], q[1], q[2], q[3])
    pos = F_K[:3, -1]
    print('Position of end effector',pos)  
    print ('Position of Target', pos_d)  
    #pos = self.detect_end_effector(image)
    # Aproach target from below 
    pos_d[2] =pos_d[2] -0.2
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    print('self error_d ', self.error_d)
    # estimate error
    self.error = pos_d-pos
    print('self error', self.error)
    #q = self.detect_joint_angles(image) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.get_jacobian(q[0], q[1], q[2], q[3]))  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    q_d = self.adjust_angles(q_d)
    return q_d, pos

  def adjust_angles(self,q_d):
    angle_list = []
    for angle in (q_d):
      angle = np.arctan2(sin(angle), cos(angle))    	
      angle_list.append(angle)
    #print(angle_list)
    q_d = np.array(angle_list)
    return q_d
        


  # Recieve data from camera 1, process it, and publish
  def callback(self,image1, image2, angles):
#  def callback(self,image1, image2, angles,target_prev):
    # Recieve the image   
    try:
    
      ### Initializastion move robot to specific position ###
      t = (rospy.get_time() - self.time_trajectory)
      print(t)
      if (t) < 0.3:
      	self.joint_2_init = Float64()
      	self.joint_2_init.data = 0
      	self.robot_joint2_pub.publish(self.joint_2_init)      	
      	self.joint_4_init = Float64()
      	self.joint_4_init.data = 0
      	self.robot_joint4_pub.publish(self.joint_4_init)
      	rospy.sleep(0.6)  
      
      image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      #im = cv2.imshow("Image1", image1)
      #cv2.waitKey(1)
      image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
      #im2 = cv2.imshow("Image2", image2)
      

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
        
#    print(target_prev.data[0])
#    F_K = self.F_K_full(angles.position[0], angles.position[1], angles.position[2], angles.position[3])
#    end_position = F_K[:3, -1]
#    print(end_position)
    #jacobian = self.get_jacobian(angles.position[0], angles.position[1], angles.position[2], angles.position[3])
    #print(jacobian)
    
    y_2nd_joint, z_2nd_joint_1 = self.detect_blue(image1)
    x_2nd_joint, z_2nd_joint_2 = self.detect_blue(image2)
    z_2nd_joint = 0.5*(z_2nd_joint_1+z_2nd_joint_2)
    pos_2nd_joint = np.array([x_2nd_joint, y_2nd_joint, z_2nd_joint])    
    px_to_meters = self.get_px_meters(pos_center, pos_2nd_joint)
    
    try:
      y,z1 = self.detect_sphere_pos(image1)
      x,z2 = self.detect_sphere_pos(image2)
      if z1 == -1:
        z_final = z2
      elif z2 == -1:
        z_final = z1
      else:
        z_final = 0.5*(z1+z2)       
      pos_sphere = np.array([x,y,z_final])
    
      pos_sphere_pxl_global, pos_sphere_meter_global = self.calc_pos_global_pxlsAndMeters(px_to_meters, pos_center, pos_sphere)
      distance_to_center = np.sqrt(np.sum([i**2 for i in pos_sphere_meter_global]))
### Control stuff###
      q = np.array([angles.position[0], angles.position[1], angles.position[2], angles.position[3]])    
      q_d, end_position = self.control_closed(pos_sphere_meter_global, q)
      print ('q_d ', q_d)
      self.joint1=Float64()
      self.joint1.data= q_d[0]
      self.joint2=Float64()
      self.joint2.data= q_d[1]
      self.joint3=Float64()
      self.joint3.data= q_d[2]
      self.joint4=Float64()
      self.joint4.data= q_d[3]  
### Publish Results
      self.end_position = Float64MultiArray()
      self.end_position.data = end_position
      self.end_pos.publish(self.end_position)   
      self.pos_sphere = Float64MultiArray()
      self.pos_sphere.data = pos_sphere_meter_global
      self.target_image.publish(self.pos_sphere)   

      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
###############################################################
    except :
      #move robot a bit 
      self.joint1=Float64()
      self.joint1.data= angles.position[0] +0.1
      self.joint2=Float64()
      self.joint2.data= angles.position[1] +0.1
      self.joint3=Float64()
      self.joint3.data= angles.position[2] +0.1
      self.joint4=Float64()
      self.joint4.data= angles.position[3] +0.1 
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)


# call the class
def main(args):
  img_cvt = image_converter()
  ### Initiate Robot at specific angles ###
  try:
    ts = message_filters.ApproximateTimeSynchronizer([img_cvt.image_sub1, img_cvt.image_sub2, img_cvt.angles], 10, 0.1)
    #ts = message_filters.ApproximateTimeSynchronizer([img_cvt.image_sub1, img_cvt.image_sub2, img_cvt.angles, img_cvt.target_image_sub], 10, 0.1)
    ts.registerCallback(img_cvt.callback)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


