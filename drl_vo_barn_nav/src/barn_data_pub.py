#!/usr/bin/env python3
import numpy as np
from random import choice
import rospy
import tf
from barn_msgs.msg import BARN_data
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from scipy.optimize import linprog
from sensor_msgs.msg import LaserScan

# parameters:
NUM_TP = 10 # the number of timestamps

class BARNData:
    # Constructor
    def __init__(self):
        
        self.scan = [] #np.zeros(720)
        self.goal_cart = np.zeros(2)
        self.scan_size_des = 720
        self.scan_tmp = np.zeros(self.scan_size_des)
        self.scan_angle_increment_des = 2 * np.pi / (self.scan_size_des - 1)

	    # timer:
        self.timer = None
        self.rate = 20  # 20 Hz velocity controller
        self.ts_cnt = 0  # maximum 7 timesteps

        # ros:
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.goal_sub = rospy.Subscriber("cnn_goal", Point, self.goal_callback)
        self.barn_data_pub = rospy.Publisher('barn_data', BARN_data, queue_size=1, latch=False)
    
    # Callback function for the scan measurement subscriber
    def scan_callback(self, laserScan_msg):
        # print("laserScan_msg: ", laserScan_msg)
        # print("len(laserScan_msg.ranges): ",len(laserScan_msg.ranges))

        max_range = laserScan_msg.range_max
        angle_increment_orig = laserScan_msg.angle_increment
        range_size = len(laserScan_msg.ranges)

        # get the laser scan data:
        self.scan_tmp = np.zeros(self.scan_size_des)
        scan_data = np.array(laserScan_msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = max_range + 0.01
        scan_data[np.isinf(scan_data)] = max_range + 0.01
        for i in range(0, self.scan_size_des):
            theta = (i - self.scan_size_des/2) * self.scan_angle_increment_des
            
            idx_low = int(np.floor( (theta + np.pi) / angle_increment_orig))
            # print("orig idx_low: ", idx_low)
            idx_low = np.max([0, idx_low])
            # print("clip idx_low: ", idx_low)

            theta_low = (idx_low - range_size/2) * angle_increment_orig
            # print("theta_low: ", theta_low)

            idx_high = int(np.ceil( (theta + np.pi) / angle_increment_orig))
            # print("orig idx_high: ", idx_high)

            idx_high = np.min([range_size - 1, idx_high])
            # print("clip idx_high: ", idx_high)

            theta_high = (idx_high - range_size/2) * angle_increment_orig
            # print("theta_high: ", theta_high)

            # print("scan_data[idx_low]: ", scan_data[idx_low])
            # print("scan_data[idx_high]: ", scan_data[idx_high])

            self.scan_tmp[i] = scan_data[idx_low] + ((theta - theta_low) / (0.000000000001 + theta_high - theta_low)) * (scan_data[idx_high] - scan_data[idx_low])

        # self.scan_tmp = scan_data

        # start the timer if this is the first path received
        if self.timer is None:
            self.start()

    # Callback function for the current goal subscriber
    def goal_callback(self, goal_msg):
        # Cartesian coordinate:
        self.goal_cart = np.zeros(2)
        self.goal_cart[0] = goal_msg.x
        self.goal_cart[1] = goal_msg.y

        # start the timer if this is the first path received
        if self.timer is None:
            self.start()
          

    # Start the timer that calculates command velocities
    def start(self):
        # initialize timer for controller update
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)

     # function that runs every time the timer finishes to ensure that velocity commands are sent regularly
    def timer_callback(self, event):  
        self.scan.append(self.scan_tmp.tolist())

        self.ts_cnt = self.ts_cnt + 1
        if(self.ts_cnt == NUM_TP): 
            # publish cnn data:
            barn_data = BARN_data()
            barn_data.scan = [float(val) for sublist in self.scan for val in sublist]
            barn_data.goal_cart = self.goal_cart
            self.barn_data_pub.publish(barn_data)

            # reset the position data list:
            self.ts_cnt = NUM_TP-1
            self.scan = self.scan[1:NUM_TP]

if __name__ == '__main__':
    try:
        rospy.init_node('barn_data')
        BARNData()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
