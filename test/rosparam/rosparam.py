#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String

def topic_py():

    # initialize node
    rospy.init_node('topic_py', anonymous=True)
    
    for i in range(1, 10):
        if rospy.has_param('param' + str(i)):
            rospy.Publisher(rospy.get_param('param' + str(i)), String, queue_size=10)

    for i in range(1, 10):
        if rospy.has_param('~private_param' + str(i)):
            rospy.Publisher(rospy.get_param('~private_param' + str(i)), String, queue_size=10)

    if rospy.has_param('/param_list'):
        for param in rospy.get_param('/param_list'):
            rospy.Publisher(paramm, String, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        topic_py()
    except rospy.ROSInterruptException:
        pass
