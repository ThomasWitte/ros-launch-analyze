#!/usr/bin/python2

import aspectlib
import rospy
import sys

from websocket import create_connection

print('open ws')
ws = create_connection("ws://localhost:34005/")

@aspectlib.Aspect
def wrap_publisher(self, topic, datatype, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
    ws.send('<<advertise>> ' + rospy.names.resolve_name(topic) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

@aspectlib.Aspect
def wrap_subscriber(self, topic, datatype, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
    ws.send('<<subscribe>> ' + rospy.names.resolve_name(topic) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

@aspectlib.Aspect
def wrap_service_advertise(self, name, datatype, handler, buff_size=65536):
    ws.send('<<advertiseService>> ' + rospy.names.resolve_name(name) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

@aspectlib.Aspect
def wrap_service_subscribe(self, name, datatype, persistent=False, headers=None):
    ws.send('<<serviceClient>> ' + rospy.names.resolve_name(name) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    yield

@aspectlib.Aspect
def wrap_wait_for_service(service, timeout=None):
    yield aspectlib.Return

@aspectlib.Aspect
def wrap_service_proxy_wait_for_service(self, timeout=None):
    yield aspectlib.Return


aspectlib.weave(rospy.Publisher.__init__, wrap_publisher)
aspectlib.weave(rospy.Subscriber.__init__, wrap_subscriber)
aspectlib.weave(rospy.Service.__init__, wrap_service_advertise)
aspectlib.weave(rospy.ServiceProxy.__init__, wrap_service_subscribe)
aspectlib.weave(rospy.ServiceProxy.wait_for_service, wrap_service_proxy_wait_for_service)
aspectlib.weave(rospy.wait_for_service, wrap_wait_for_service)

# print('loaded aspects')

sys.argv = sys.argv[1:]

exec(open(sys.argv[0]).read(), globals())

print('close ws')
ws.close()
