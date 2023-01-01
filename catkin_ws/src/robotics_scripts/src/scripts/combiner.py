#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
from robotics_project.msg import custom_msg

class combiner:

    def __init__(self):

        self.msg = custom_msg() 
        self.publisher = rospy.Publisher('/laser_odom', custom_msg, queue_size=1)
        self.laser = message_filters.Subscriber('/scan_multi', LaserScan)
        self.odom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    def sensorCallback(self,laser,odom):
        self.msg.laser_msg = laser
        self.msg.odom_msg = odom
        self.publisher.publish(self.msg)

    def combine(self):
        synchronizer = message_filters.ApproximateTimeSynchronizer([self.laser,self.odom],1,0.0001,allow_headerless=True)
        synchronizer.registerCallback(self.sensorCallback)


if __name__ == '__main__':

        rospy.init_node('combine_lasers_and_odom')
        rospy.loginfo('starting combiner node')
        rate = rospy.Rate(2)
        combiner = combiner()
        while not rospy.is_shutdown():
            combiner.combine()
            rate.sleep()
        rospy.spin()