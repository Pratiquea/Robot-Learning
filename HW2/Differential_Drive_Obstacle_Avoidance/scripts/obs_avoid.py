#!/usr/bin/env python
import rospy # Python library for ROS
import numpy as np
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

class DiffDrive:
    def __init__(self):
        self.thresh = 2.0 # Laser scan range threshold
        self.move = Twist() # Creates a Twist message type object
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserscan_cb) 

    def laserscan_cb(self,data):
        segment = int(len(data.ranges)*0.4)
        middle = int(len(data.ranges)*0.5)
        area1 = np.median(data.ranges[:segment])
        area2 = np.median(data.ranges[middle-segment:middle+segment])
        area3 = np.median(data.ranges[-segment:])
        print '-------------------------------------------'
        print 'Median value of Range data between first 50 values:   {}'.format(area1)
        print 'Median value of Range data between index 100 and 150:  {}'.format(area2)
        print 'Median value of Range data between last 50 values: {}'.format(area3)
        print '-------------------------------------------'
        if area1>self.thresh and area2>self.thresh and area3>self.thresh: 
            self.move.linear.x = 0.5 # move forward
            self.move.angular.z = 0.0 # No angular velocity
        elif area1>self.thresh and area2<self.thresh and area3<self.thresh: 
            self.move.linear.x = -0.3 # Move backwards
            self.move.angular.z = -0.5 # rotate counter-clock wise
        elif area1<self.thresh and area2<self.thresh and area3>self.thresh: 
            self.move.linear.x = -0.3 # Move backwards
            self.move.angular.z = 0.5 # rotate counter-clock wise
        else:
            self.move.linear.x = -0.3 # Move backward
            self.move.angular.z = 0.5 # rotate clockwise at 0.5 rad/sec
        self.vel_pub.publish(self.move) # publish the move object


    def avoid_obstacle(self):
        rospy.init_node('obstacle_avoidance_node') # Initializes a node
        r = rospy.Rate(50) # 50hz
        while(not rospy.is_shutdown()):
            r.sleep()

def main():
    robot = DiffDrive()
    robot.avoid_obstacle()

if __name__ == '__main__':
    main()