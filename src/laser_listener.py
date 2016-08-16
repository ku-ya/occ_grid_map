#!/usr/bin/env python
import roslib
import rospy
import sensor_msgs.msg
from nav_msgs.msg import Odometry

def callback_odom(msg):
    # print (msg.pose.pose.position)
    pass


def callback(data):
    print data
    # pass
def laser_listener():
    # pass
    rospy.init_node('laser_listener', anonymous=True)
    # rospy.Subscriber("/sensor_msgs/LaserScan",sensor_msgs.msg.LaserScan,callback,20)
    rospy.Subscriber("base_scan_1",sensor_msgs.msg.LaserScan,callback)
    rospy.Subscriber("odom",Odometry,callback_odom)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()
