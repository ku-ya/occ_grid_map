#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys

twist = Twist()

def keyboard():
    pub = rospy.Publisher('/mobile_base/commands/velocity',Twist, queue_size=10)
    rospy.init_node('teleop_py',anonymous=True)
    rate = rospy.Rate(10)
    k = 1
    while not rospy.is_shutdown() and k < 300:
      twist.linear.x = 1.0
      twist.angular.z = 0.02
      twist.linear.y = 0.0
      pub.publish(twist)
      rate.sleep()
      k +=1
    k = 1
    while not rospy.is_shutdown() and k < 10:
      twist.linear.x = 0.0
      twist.angular.z = 0.1
      twist.linear.y = 0.0
      pub.publish(twist)
      rate.sleep()



if __name__=='__main__':
  try:
    keyboard()
  except rospy.ROSInterruptException:
    pass
