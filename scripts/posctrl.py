#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
from pymavlink import mavutil
from tf.transformations import euler_from_quaternion
from offboard_comm import *
from lidar_process import *


def main():
    rospy.init_node('setpoint_node', anonymous=True)

    print("node started")
    agent = OffboardCtrl()
    lidar = Lidar()
    print("Class loaded")
    agent.wait_for_topics(1)
    print("Topic loaded")

    # pose
    # Yaw, X, Y, Z
    # Waypoints
    pos_takeoff = [0, 0, 0, 2]
    pos1 = [0, 3, 0, 0]
    pos2 = [0, 1, 1, -0.5]
    pos3 = [0, -3, -2, 1.5]
    loop_rate = rospy.Rate(10)
      
        
    while not rospy.is_shutdown():
        # Possible direction & distance - drone coordinate
        # Set step 1 m, drone radius 0.65 m
        possibles = lidar.scan_possible(dist=2.0, limit=0.73, ret=True)
        
        relx = pos1[1] - agent.local_position.pose.position.x
        rely = pos1[2] - agent.local_position.pose.position.y
        rel_pos = np.array([relx, rely])
        
        rel_yaw = np.arctan2(rel_pos[1], rel_pos[0])
        current_yaw = euler_from_quaternion([agent.local_position.pose.orientation.x, agent.local_position.pose.orientation.y, agent.local_position.pose.orientation.z, agent.local_position.pose.orientation.w])[2]
        
        directions = np.deg2rad(possibles[:,0])
        dir_errs = directions - rel_yaw
        straight_flag = False
        if np.amin(dir_errs) < np.deg2rad(5.0):
            straight_flag = True
            
        print(np.rad2deg(np.amin(dir_errs)), straight_flag, len(directions))
        
        loop_rate.sleep()
        
    
        
    
    """
    print("Test trajectories")

    # agent.set_mode_srv(custom_mode='OFFBOARD')
    # agent.set_arming_srv(True)

    # agent.reach_position(pos_takeoff, 30)

    agent.halt()
    print("Set desired as current position - Init")

    # 3 sec hovering
    hov_time = 30  # 10 Hz update

    agent.move_to(pos_takeoff)
    while (agent.is_at_position(pos_takeoff, agent.radius) == False):
        loop_rate.sleep()
    print("Take off Done")

    for _ in range(hov_time):
        loop_rate.sleep()

    # agent.dronet_mv(pos_takeoff)
    agent.dronet_mv(pos1)
    agent.dronet_mv(pos2)
    agent.dronet_mv(pos3)

    for _ in range(hov_time):
        loop_rate.sleep()

    landing_condition = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    agent.set_mode_srv(custom_mode="AUTO.LAND")
    while (agent.extended_state.landed_state != landing_condition):
        loop_rate.sleep()

    print("Landing done")

    agent.set_arming_srv(False)
    """


if __name__ == '__main__':
    main()
