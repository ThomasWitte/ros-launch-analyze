#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String

if __name__ == '__main__':
    topic_name = 'topic_' + str(random.randint(0,5))
    rospy.init_node('random_node')
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    pub.publish('hello')
    rospy.spin()
