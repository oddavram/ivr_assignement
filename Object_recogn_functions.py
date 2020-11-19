import cv2
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import figure
from math import pi

directory = 'C:/Edinburgh/Courses/Imaging and robotics/Assignement/part1_2 object recogn/specific angles test/'
os.chdir(directory)
image_1_fname = 'image1_angle4_90.png'
image_2_fname = 'image2_angle4_90.png'

#Test Cases
#image1_copy.png
#image2_copy.png
#image1_copy (another copy).png
#image1_copy (another copy).png
#image1_copy (3rd copy).png
#image1_copy (4th copy).png

image_1 = cv2.imread(image_1_fname,1)
image_2 = cv2.imread(image_2_fname,1)


def get_frame_pixel_pos(image):
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
    # Plot image for verification

    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_Y, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return (centroid_Y)


def get_2nd_joint_pos(image):
    ## Get Frame position with respect to the image ###
    ## apply yellow mask
    lowerBound_B = (100, 0, 0);
    upperBound_B = (255, 0, 0);
    mask_B = cv2.inRange(image, lowerBound_B, upperBound_B);
    kernel = np.ones((5, 5), np.uint8)
    mask_B = cv2.dilate(mask_B, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_B, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    centroid_B = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    # Plot image for verification

    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_B, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return (centroid_B)


def get_px_meters(pos_center, pos_2nd_joint):
    link_length = 2.5
    ## joint 1 only rotates around z!!
    px_difference = abs(pos_center[2] - pos_2nd_joint[2])
    return link_length / px_difference


def get_orange_pos(image):
    lowerBound_O = (0, 0, 180);
    upperBound_O = (187, 220, 250);
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_O = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
    # kernel = np.ones((5, 5), np.uint8)
    # mask_O = cv2.dilate(mask_O, kernel, iterations=3)
    contours, hierarchy = cv2.findContours(mask_O, 1, 2)
    # Plot image for verification
    figure(figsize=(15, 15))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 15))
    ax1.imshow(cv2.cvtColor(mask_O, cv2.COLOR_BGR2RGB))
    ax2.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    return contours


def get_object_features(contours):
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


def recognize_sphere(aspect_ratios, rectness, circless):
    sphericity = [aspect_ratios[idx] - rectness[idx] + circless[idx] for idx in range(len(aspect_ratios))]
    if max(sphericity) < 0.6:
        print('No sphere was detected! Sphere behind Robot!')
        return -1
    else:
        return sphericity.index(max(sphericity))


def get_pos_sphere(contour, sphere_contour_idx):
    cnt = contour[sphere_contour_idx]
    M = cv2.moments(cnt)
    centroid_sphere = [M['m10'] / M['m00'], M['m01'] / M['m00']]
    return centroid_sphere


def detect_sphere_pos(image):
    contours = get_orange_pos(image)
    aspect_ratios, rectness, circless = get_object_features(contours)
    sphere_contour_idx = recognize_sphere(aspect_ratios, rectness, circless)
    if sphere_contour_idx == -1:
        return (0, -1)
    return get_pos_sphere(contours, sphere_contour_idx)

def calculate_pos_sphere(pos_sphere, pos_center, px_to_meters):
    dist = np.sum([(pos_center[i]-pos_sphere[i])**2 for i in range(len(pos_center))])
    dist = px_to_meters*(np.sqrt(dist))
    pos_sphere_meters = [pos_sphere[i] - pos_center[i] for i in range(len(pos_center))]
    return (pos_sphere_meters, dist)


x, z1 = detect_sphere_pos(image_1)
y, z2 = detect_sphere_pos(image_2)
if z1 == -1:
    z_final = z2
elif z2 == -1:
    z_final = z1
else:
    z_final = 0.5 * (z1 + z2)
pos_sphere = np.array([x, y, z_final])

x_center, z_center_1 = get_frame_pixel_pos(image_1)
y_center, z_center2 = get_frame_pixel_pos(image_2)
z_center = 0.5 * (z_center_1 + z_center2)
pos_center = np.array([x_center, y_center, z_center])

x_2nd_joint, z_2nd_joint_1 = get_2nd_joint_pos(image_1)
y_2nd_joint, z_2nd_joint_2 = get_2nd_joint_pos(image_2)
z_2nd_joint = 0.5 * (z_2nd_joint_1 + z_2nd_joint_2)
pos_2nd_joint = np.array([x_2nd_joint, y_2nd_joint, z_2nd_joint])
px_to_meters = get_px_meters(pos_center, pos_2nd_joint)

pos_sphere_center, distance_to_center = calculate_pos_sphere(pos_sphere, pos_center, px_to_meters)
pos_sphere_center_meters = [px_to_meters * position for position in pos_sphere_center]

print('Sphere Pos = ', x, y, z_final)
print('Center Pos = ', x_center, y_center, z_center)
print('Second Joint Pos=', x_2nd_joint, y_2nd_joint, z_2nd_joint)
print('Pxs to meters value: ', px_to_meters)
print('Position of Sphere relative to the center (Pxs): ', pos_sphere_center)
print('Position of Sphere relative to the center (meters): ', pos_sphere_center_meters)
print('Distance of sphere to the center: ', distance_to_center)

