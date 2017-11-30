#!/usr/bin/python2

import aspectlib
import rospy
import sys

@aspectlib.Aspect
def wrap_publisher(self, topic, datatype, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
    print('<<advertise>> ' + topic + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

@aspectlib.Aspect
def wrap_subscriber(self, topic, datatype, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
    print('<<subscribe>> ' + topic + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

aspectlib.weave(rospy.Publisher.__init__, wrap_publisher)
aspectlib.weave(rospy.Subscriber.__init__, wrap_subscriber)

# print('loaded aspects')

sys.argv = sys.argv[1:]

exec(open(sys.argv[0]).read(), globals())
