#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from robotics_project.msg import custom_msg
import tf

class extended_scan:

    def __init__(self):

        self.publisher = rospy.Publisher('/robot/all_in_one', LaserScan, queue_size=10)
        self.scan_front = None
        self.scan_rear = None
        self.odom = None

    def laser_1_callback(self, data1):
        # print (data1)
        # print 'sai'
        self.scan_front = data1

        return self.scan_front

    def laser_2_callback(self, data2):
        # print (data2)
        # print 'sai1'
        self.scan_rear = data2

        return self.scan_rear

    def odom_callback(self, msg):
        # print (msg)
        # print 'sai1'
        self.odom = msg

        return self.odom

    def newScan_pub(self):
        while not rospy.is_shutdown():
            # listener = tf.TransformListener()
            # (trans,rot)=listener.lookupTransform('robot_base_link','robot_front_laser_link',rospy.rostime())
            # print(trans,rot)
            rospy.Subscriber('/robot/front_laser/scan', LaserScan, self.laser_1_callback)
            rospy.Subscriber('/robot/rear_laser/scan', LaserScan, self.laser_2_callback)
            rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)
            newdata = LaserScan()
            newcustom= custom_msg()
            if self.scan_front == None or self.scan_rear == None:
                rospy.loginfo('got None data')
                pass
            else:
                newdata.header.seq = self.scan_front.header.seq
                newdata.header.stamp = self.scan_front.header.stamp
                newdata.header.frame_id = 'robot_front_laser_base_link'
                newdata.angle_min = self.scan_front.angle_min
                newdata.angle_max = self.scan_front.angle_max + -(self.scan_rear.angle_min) + self.scan_rear.angle_max
                newdata.angle_increment = self.scan_front.angle_increment
                newdata.time_increment = self.scan_front.time_increment
                newdata.scan_time = self.scan_front.scan_time
                newdata.range_min = self.scan_front.range_min
                newdata.range_max = self.scan_front.range_max
                newdata.ranges = self.scan_front.ranges + self.scan_rear.ranges
                newdata.intensities = self.scan_front.intensities
                print (len(self.scan_front.ranges))
                print (len(self.scan_rear.ranges))
                # print len(newdata.ranges)
                newcustom.laser_msg = newdata
                newcustom.odom_msg = self.odom
                self.publisher.publish(newdata)
                # rospy.spin()
if __name__ == '__main__':

    rospy.init_node('extended_scan_publisher')
    rospy.loginfo(' starting extended_scan_publishe node')

    ES = extended_scan()
    ES.newScan_pub()
    # rospy.spin()