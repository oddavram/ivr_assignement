#### forward Kinematics ###

import cv2
import numpy as np
import os
from matplotlib import pyplot as plt
from matplotlib.pyplot import figure
import math

theta1 = 0
theta2 = math.pi / 2
theta3 = math.pi / 2
theta4 = math.pi / 2

input_joints = [theta1, theta2, theta3, theta4]


def get_end_position(input_joints):
    theta1 = input_joints[0]
    theta2 = input_joints[1]
    theta3 = input_joints[2]
    theta4 = input_joints[3]

    T1 = np.array([
        [math.sin(theta1), 0, math.cos(theta1), 0],
        [-math.cos(theta1), 0, math.sin(theta1), 0],
        [0, -1, 0, 2.5],
        [0, 0, 0, 1]
    ])

    T2 = np.array([
        [math.sin(theta2), 0, -math.cos(theta2), 0],
        [-math.cos(theta2), 0, -math.sin(theta2), 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    T3 = np.array([
        [-math.cos(theta3), 0, -math.sin(theta3), 3.5 * math.cos(theta3)],
        [-math.sin(theta3), 0, math.cos(theta3), 3.5 * math.sin(theta3)],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    T4 = np.array([
        [math.cos(theta4), -math.sin(theta4), 0, -3 * math.cos(theta4)],
        [math.sin(theta4), math.cos(theta4), 0, -3 * math.sin(theta4)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T1_T2 = np.matmul(T1, T2)
    T1_T2_T3 = np.matmul(T1_T2, T3)
    Tran_mat = np.matmul(T1_T2_T3, T4)
    pos = Tran_mat[:, -1]
    return pos


pos = get_end_position(input_joints)

print(pos)
