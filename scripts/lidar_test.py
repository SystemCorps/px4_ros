#!/usr/bin/env python

import rospy
import math
import numpy as np
from turtle_posctrl import *
from lidar_process import *


def main():

    rospy.init_node('burgur_test', anonymous=True)

    lidar = Lidar()
    burger = PosCtrl()

    rate = 10
    loop = rospy.Rate(rate)

    for _ in range(rate*3):
        loop.sleep()

    goal = [-6,1]

    while(not rospy.is_shutdown()):
        lidar.scan_possible()


        loop.sleep()


