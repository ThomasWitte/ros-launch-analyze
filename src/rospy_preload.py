#!/usr/bin/python2

import aspectlib
import rospy
import sys

from websocket import create_connection

ws = create_connection("ws://localhost:34005/")

# Topics can be created before the node is initialized. In this case, the names might not resolve correctly.
topics_before_initialization = []

@aspectlib.Aspect
def wrap_publisher(self, topic, datatype, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
    if rospy.core.is_initialized():
        ws.send('<<advertise>> ' + rospy.names.resolve_name(topic) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    else:
        topics_before_initialization.append(('<<advertise>> ', topic, ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__))
    yield

@aspectlib.Aspect
def wrap_subscriber(self, topic, datatype, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
    if rospy.core.is_initialized():
        ws.send('<<subscribe>> ' + rospy.names.resolve_name(topic) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    else:
        topics_before_initialization.append(('<<subscribe>> ', topic, ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__))
    yield

@aspectlib.Aspect
def wrap_service_advertise(self, name, datatype, handler, buff_size=65536):
    if rospy.core.is_initialized():
        ws.send('<<advertiseService>> ' + rospy.names.resolve_name(name) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    else:
        topics_before_initialization.append(('<<advertiseService>> ', name, ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__))
    yield

@aspectlib.Aspect
def wrap_service_subscribe(self, name, datatype, persistent=False, headers=None):
    if rospy.core.is_initialized():
        ws.send('<<serviceClient>> ' + rospy.names.resolve_name(name) + ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__)
    else:
        topics_before_initialization.append(('<<serviceClient>> ', name, ' ' + datatype.__module__.split('.')[0] + '/' + datatype.__name__))
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

# hack, as directly weaving into rospy.init_node does not work for some reason
orig_init_node = rospy.init_node
def init_node(name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False, disable_signals=False, xmlrpc_port=0, tcpros_port=0):
    global topics_before_initialization
    result = orig_init_node(name, argv=argv, anonymous=anonymous, log_level=log_level, disable_rostime=disable_rostime, disable_rosout=disable_rosout, disable_signals=disable_signals, xmlrpc_port=xmlrpc_port, tcpros_port=tcpros_port)
    for (prefix, name, suffix) in topics_before_initialization:
        ws.send(prefix + rospy.names.resolve_name(name) + suffix)
    topics_before_initialization = []
    return result
rospy.init_node = init_node

# print('loaded aspects')

sys.argv = sys.argv[1:]

exec(open(sys.argv[0]).read(), globals())

ws.close()
