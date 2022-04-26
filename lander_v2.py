"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

import numpy as np
import time
import swarmlib
from swarmlib import Drone, mocap_object
import rospy
import math

from pycrazyswarm import Crazyswarm
from crazyswarm.msg import FullState
from crazyswarm.msg import Position
from geometry_msgs.msg import TransformStamped

import matplotlib.pyplot as plt
from potential_fields import *
from tools import *
import os
from multiprocessing import Process

TAKEOFF_DURATION = 2.
HOVER_DURATION = 1.0


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    rate = rospy.Rate(10)
    cf = []

    cf.append(swarm.allcfs.crazyflies[0])
    cf[0].takeoff(targetHeight=1.3, duration=TAKEOFF_DURATION)
    for i in [1, 2]:
        cf.append(swarm.allcfs.crazyflies[i])
        cf[i].takeoff(targetHeight=1.3, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)


    cf[0].goTo(goal=np.array([0.0, 0.0, 1.3]), duration=2, yaw=0)
    cf[1].goTo(goal=np.array([-0.5, 0.5, 1.3]), duration=2, yaw=0)
    cf[2].goTo(goal=np.array([-0.5, -0.5, 1.3]), duration=2, yaw=0)
    timeHelper.sleep(2)



    allcfs = swarm.allcfs

    superviser = Drone('cf1', [0.0, 0., 0.2], cf[0])
    drones = [superviser, Drone('cf21', [-0.5, 0.5, 0.2]), Drone('cf19', [-0.5, -0.5, 0.2])]

    land_shifts = [[0.0, 0., 0.05], [0.2, 0.15, 0.05], [0.2, -0.15, 0.05]]

    while not rospy.is_shutdown():
        rospy.Subscriber('/pos', TransformStamped, superviser.cam_data)

        for dr, i in zip(drones, range(1)):
            dr.position()
            dr.follower(superviser.pos_goal)
            if dr.land_request == 1:
                dr.shift = land_shifts[i]

        if all([dr.shutdown for dr in drones]):
            rospy.signal_shutdown("Drone Landed gracefully!")
            break

        for dr in drones:
            dr.fly()



