#!/usr/bin/env/python3
from robot_service import RobotService
import _thread
import numpy as np
import cv2
import os

from robot_object import RobotObject

import rospy
from sst_interfaces.srv import *

# target_num = 0

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))



def imageToArray(img_path = '../maps/map.png'):
    image = cv2.imread(img_path)
    ox = []
    oy = []
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if np.sum(image[i][j]) == 0:
                ox.append(j)
                oy.append(i)

    return ox, oy


def main():
    rospy.init_node('pr_vis_node')
    print(__file__ + " start!!")

    robot_0 = RobotObject('robot_0')
    robot_1 = RobotObject('robot_1')

    ox = [] 
    oy = []

    map_path = os.path.join(BASE_DIR, 'src/map.png')
    ox, oy = imageToArray(map_path)

    _thread.start_new_thread(RobotService, (robot_0, ox, oy))
    _thread.start_new_thread(RobotService, (robot_1, ox, oy))

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()