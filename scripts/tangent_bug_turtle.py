#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates




class BugTurtle:
    def __init__(self):

        self.goalVisible = False
        self.discThresh = 0.2

        # LiDAR
        self.width = 0.65
        self.lidar_max = 3.3  # Maximum range of LiDAR
        self.lidar_scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.lidar_data = LaserScan()
        self.obs_dist_thresh = 0.4
        self.Maginot = 0.3
        self.front_deg = 45
        self.front_obs_search = [5, 35]
        self.goal_thresh = 0.1
        self.isReach = False
        self.candidates = []
        self.filtered_data = np.zeros(360)

        # Simulation
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.burger_pos = Pose()
        self.current_yaw = 0


        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.desired_vel = Twist()
        self.desired_vel.linear.x = 0
        self.desired_vel.angular.y = 0
        self.maxLinVel = 0.2
        self.maxYawVel = 0.5
        self.only_yaw = np.pi / 4


        # relative position
        self.rel_pos = [0, 0]

        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()




    def scan_possible(self, dist=1.0, limit=0.65, ret=False):
        temp = np.arctan2(dist, limit)
        theta = np.pi/2 - temp
        theta_deg = np.rad2deg(theta)
        theta_int = int(math.ceil(theta_deg))


        possible_head = []
        for i in range(len(self.lidar_data.ranges)):
            if i < theta_int:
                scan_pos = self.filtered_data[:theta_int+i+1]
                scan_neg = self.filtered_data[-(theta_int-i):]
                scan = np.concatenate((scan_neg, scan_pos), axis=None)


            elif i >= theta_int and i < len(self.lidar_data.ranges)-theta_int:
                scan = self.filtered_data[(i-theta_int):(i+theta_int+1)]

            else:
                scan_pos = self.filtered_data[(i-theta_int):]
                scan_neg = self.filtered_data[:theta_int-(360-i)+1]
                scan = np.concatenate((scan_pos, scan_neg), axis=None)

            #self.scantest.append(scan)
            min_dist = np.amin(scan)
            if min_dist >= dist+limit:
                avg_dist = np.mean(scan)
                possible_head.append([i, min_dist])

        self.candidates = possible_head

        if ret:
            return possible_head


    def gazebo_callback(self, data):
        self.burger_pos = data.pose[-1]
        self.current_yaw = euler_from_quaternion([self.burger_pos.orientation.x,
                                                  self.burger_pos.orientation.y,
                                                  self.burger_pos.orientation.z,
                                                  self.burger_pos.orientation.w])[2]

    def lidar_callback(self, data):
        self.lidar_data = data

        for i in range(len(self.lidar_data.ranges)):
            if self.lidar_data.ranges[i] == 0.0 or self.lidar_data.ranges[i] == np.inf:
                self.filtered_data[i] = self.lidar_max
            else:
                self.filtered_data[i] = self.lidar_data.ranges[i]


    def send_vel(self):
        loop = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.desired_vel)
            loop.sleep()


    def tanBug(self, goal=[0,0]):
        bugLoop = rospy.Rate(5)

        while (not rospy.is_shutdown() and not self.isReach):

            self.rel_pos[0] = goal[0] - self.burger_pos.position.x
            self.rel_pos[1] = goal[1] - self.burger_pos.position.y
            self.rel_pos = np.array(self.rel_pos)

            angle2goal = np.arctan2(self.rel_pos[1], self.rel_pos[0])
            dist2goal = np.linalg.norm(self.rel_pos)
            yaw_err = angle2goal - self.current_yaw

            lidar_idx = int((angle2goal - self.lidar_data.angle_min)/self.lidar_data.angle_increment)
            #print(yaw_err, dist2goal)

            # Check if we can see the goal directly
            if (self.lidar_data.ranges[lidar_idx] >= self.lidar_max):
                self.goalVisible = True
                if dist2goal >= self.maxLinVel / 2:
                    self.desired_vel.linear.x = self.maxLinVel
                else:
                    self.desired_vel.linear.x = self.maxLinVel * dist2goal / (self.maxLinVel / 2)

                if abs(yaw_err) >= self.maxYawVel / 2:
                    self.desired_vel.angular.z = np.sign(yaw_err) * self.maxYawVel
                else:
                    self.desired_vel.angular.z = self.maxYawVel * yaw_err / (self.maxYawVel / 2)

            else:
                self.goalVisible = False


            if not self.goalVisible:
                bestAngle = 0.0
                besti = 0
                bestDist = 10.0

                possibles = self.scan_possible(dist=1.0, limit=0.2, ret=True)

                if len(possibles) > 0:
                    for i in range(len(possibles)):
                        dAng = np.deg2rad(possibles[i][0])
                        discDist = possibles[i][1]
                        xDist = discDist * np.sin(dAng)
                        yDist = discDist * np.cos(dAng)
                        xyDist = np.array([xDist, yDist])
                        heurDist = np.linalg.norm(self.rel_pos - xyDist)
                        if (heurDist + discDist) < bestDist:
                            bestDist = heurDist + discDist
                            bestAngle = dAng
                            besti = int(possibles[i][0])

                """
                for i in range(len(self.lidar_data.ranges)):
                    if ( i > 0) and (abs(self.lidar_data.ranges[i] - self.lidar_data.ranges[i-1]) > self.discThresh):
                        discDist = self.lidar_data.ranges[i]

                        if discDist == float('inf'):
                            discDist = self.lidar_data.range_max
                        dAng = self.lidar_data.angle_min + i * self.lidar_data.angle_increment
                        xDist = discDist * np.sin(dAng)
                        yDist = discDist * np.cos(dAng)
                        xyDist = np.array([xDist, yDist])
                        heurDist = np.linalg.norm(self.rel_pos - xyDist)

                        if (heurDist + discDist) < bestDist:
                            bestDist = heurDist + discDist
                            bestAngle = dAng
                            besti = i
                """

                yaw_err = bestAngle - self.current_yaw
                print(bestAngle, besti)
                """
                if abs(bestAngle) >= self.maxYawVel / 2:
                    self.desired_vel.linear.x = self.maxLinVel
                    self.desired_vel.angular.z = np.sign(bestAngle) * self.maxYawVel
                elif abs(bestAngle) < self.maxYawVel / 2:
                    self.desired_vel.linear.x = self.maxLinVel
                    self.desired_vel.angular.z = self.maxYawVel * bestAngle / (self.maxLinVel / 2)
                 """
                if bestAngle == 0:
                    self.desired_vel.linear.x = self.maxLinVel
                    self.desired_vel.angular.z = 0
                    print("a")
                elif abs(bestAngle) >= self.only_yaw:
                    self.desired_vel.linear.x = 0
                    self.desired_vel.angular.z = np.sign(bestAngle) * self.maxYawVel
                    print("b")
                else:
                    self.desired_vel.linear.x = self.maxLinVel
                    self.desired_vel.angular.z = np.sign(bestAngle) * self.maxYawVel
                    print("c")



                # Obstacle avoidance
                if (besti > self.front_deg) and (besti < (len(self.lidar_data.ranges) - self.front_deg)):
                    if self.lidar_data.ranges[besti+10] < self.obs_dist_thresh:
                        self.desired_vel.linear.x = self.maxLinVel
                        self.desired_vel.angular.z = self.maxYawVel
                        print("d")
                    elif self.lidar_data.ranges[besti-10] < self.obs_dist_thresh:
                        self.desired_vel.linear.x = self.maxLinVel
                        self.desired_vel.angular.z = -self.maxYawVel
                        print("e")


            j = int(len(self.lidar_data.ranges) / 4) - self.front_obs_search[1]
            m = int(len(self.lidar_data.ranges) / 4) - self.front_obs_search[0]
            k = int(len(self.lidar_data.ranges) / 4) + self.front_obs_search[1]
            n = int(len(self.lidar_data.ranges) / 4) + self.front_obs_search[0]

            for i in range(j, m):
                if self.lidar_data.ranges[i] < self.Maginot:
                    self.desired_vel.linear.x = 0
                    self.desired_vel.angular.z = self.maxYawVel
                    print("f")

            for i in range(n, k):
                if self.lidar_data.ranges[i] < self.Maginot:
                    self.desired_vel.linear.x = 0
                    self.desired_vel.angular.z = -self.maxYawVel
                    print("g")

            if dist2goal <= self.goal_thresh:
                self.desired_vel.linear.x = 0
                self.desired_vel.angular.z = 0
                self.isReach = True

            print("vel", self.desired_vel.angular.z, self.desired_vel.linear.x)

            bugLoop.sleep()


def main():
    rospy.init_node('tanBug_test', anonymous=True)

    rate = 30
    loop = rospy.Rate(rate)

    bug = BugTurtle()

    # Wait
    for _ in range(rate*3):
        loop.sleep()


    goal1 = [6,-1]
    goal2 = [2,4]

    while not bug.isReach:
        bug.tanBug(goal=goal1)
        loop.sleep()


if __name__ == '__main__':
    main()

