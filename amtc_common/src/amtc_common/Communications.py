# -*- coding: utf-8 -*-


import rospy
import threading


class Subscriber:
  def __init__(self, topic, type, cb = None):
    self.mutex            = threading.Lock()
    self.msg              = type()
    self.cb               = cb
    self.last_msg_time    = rospy.Time(0)
    if self.cb is None:
      self.sub  = rospy.Subscriber(topic, type, self.input_cb)
      pass
    else:
      self.sub  = rospy.Subscriber(topic, type, self.input_nest_cb)
      pass
    pass
  def input_cb(self,msg):
    self.mutex.acquire()
    self.last_msg_time = rospy.Time.now()
    self.msg =  msg
    self.mutex.release()
    pass
  def input_nest_cb(self,msg):
    self.mutex.acquire()
    self.last_msg_time = rospy.Time.now()
    self.msg =  msg
    self.cb(msg)
    self.mutex.release()
    pass
  def set_msg(self, msg):
    self.mutex.acquire()
    self.msg = msg
    self.mutex.release()
    pass
  def get_msg(self):
    return self.msg
    pass
  def get_msg_safe(self):
    self.mutex.acquire()
    return self.msg
    pass
  def get_msg_safe_release(self):
    self.mutex.release()
    pass
  def get_last_msg_time(self):
    return self.last_msg_time
    pass
  def unregister(self):
    self.sub.unregister()
    pass
  pass

class Publisher:
  def __init__(self, topic, type, queue_size=5):
    self.pub = rospy.Publisher(topic, type, queue_size = queue_size)
    self.msg = type()
    pass
  def set_msg(self, msg):
    self.msg = msg
    pass
  def get_msg(self):
    return self.msg
    pass
  def publish(self, msg = None):
    if msg is None:
      self.pub.publish(self.msg)
      pass
    else:
      self.pub.publish(msg)
      pass
    pass
  def unregister(self):
    self.pub.unregister()
    pass
  pass

class ServiceClient:
  def __init__(self, service_topic, type):
    self.msg              = type()
    self.service_topic    = service_topic
    self.service          = rospy.ServiceProxy(service_topic, type,persistent=True)
    pass
  def wait_for_service(self):
    rospy.wait_for_service(self.service_topic)
    pass
  def call(self, request):
    return self.service( request )
    pass
  def unregister(self):
    self.service.close()
    pass

  pass

class ServiceServer:
  def __init__(self, service_topic, type, cb ):
    self.msg              = type()
    self.service_topic    = service_topic
    self.service          = rospy.Service(service_topic, type, cb)
    pass
  def unregister(self):
    self.service.shutdown()
  pass

class Timer:
  def __init__(self, duration, cb, oneshot = False, autostart = True):
    self.oneshot    = oneshot
    self.cb         = cb
    self.duration   = duration
    self.fired      = False
    self.running    = False
    self.autostart  = autostart
    if autostart == True:
      self.start()
      pass
    pass
  def stop(self):
    if self.running:
      self.timer.shutdown()
      pass
    pass
  def start(self):
    self.running  = True
    self.fired    = False
    self.timer = rospy.Timer(self.duration, self.timer_cb, self.oneshot)
    pass
  def triggered(self):
    return self.fired
    pass
  def set_period(self, duration):
    self.duration = duration
    if self.running:
      self.stop()
      if self.autostart:
        self.start()
        pass
      pass
    pass
  def running(self):
    return self.running
    pass
  def timer_cb(self,event):
    self.fired    = True
    self.cb(event)
    pass
  pass


class Communications:
  def __init__(self):
    self.publishers       = dict()
    self.subscribers      = dict()
    self.service_clients  = dict()
    self.service_servers  = dict()
    self.timers           = dict()
    self.node_configure   = NodeConfigure()
    pass
  def add_publisher(self, topic, type, keyword, queue_size = 5):
    if keyword in self.publishers:
      rospy.logerr("add_publisher: keyword "+keyword+" already used")
      return
      pass
    self.publishers[keyword] = Publisher(topic, type, queue_size)
    pass
  def remove_publisher(self, keyword):
    if keyword in self.publishers:
      self.publishers[keyword].unregister()
      del self.publishers[keyword]
      pass
    else:
      rospy.logerr("remove_publisher: keyword "+keyword+" not present")
      pass
    pass
  def publish(self, keyword, msg = None):
    if keyword in self.publishers:
      self.publishers[keyword].publish(msg)
      pass
    else:
      rospy.logerr("publish: keyword "+keyword+" not present")
      pass
    pass
  def get_publisher(self, keyword):
    if keyword in self.publishers:
      return self.publishers[keyword]
      pass
    else:
      rospy.logerr("get_publisher: keyword "+keyword+" not present")
      pass
    pass
  def add_subscriber(self, topic, type, keyword,  cb = None):
    if keyword in self.subscribers:
      rospy.logerr("add_subscriber: keyword "+keyword+" already used")
      return
      pass
    self.subscribers[keyword] = Subscriber(topic, type, cb)
    pass
  def remove_subscriber(self, keyword):
    if keyword in self.subscribers:
      self.subscribers[keyword].unregister()
      del self.subscribers[keyword]
      pass
    else:
      rospy.logerr("remove_subscriber: keyword "+keyword+" not present")
      pass
    pass
  def get_subscriber(self, keyword):
    if keyword in self.subscribers:
      return self.subscribers[keyword]
      pass
    rospy.logerr("get_subscriber: keyword "+keyword+" not present")
    pass
  def add_service_client(self, topic, type, keyword):
    if keyword in self.service_clients:
      rospy.logerr("add_service_client: keyword "+keyword+" already used")
      return
      pass
    self.service_clients[keyword] = ServiceClient(topic, type)
    pass
  def remove_service_client(self, keyword):
    if keyword in self.service_clients:
      self.service_clients[keyword].unregister()
      del self.service_clients[keyword]
      pass
    else:
      rospy.logerr("remove_service_client: keyword "+keyword+" not present")
      pass
    pass
  def get_service_client(self, keyword):
    if keyword in self.service_clients:
      return self.service_clients[keyword]
      pass
    else:
      rospy.logerr("get_service_client: keyword "+keyword+" not present")
      pass
    pass
  def call_service(self, keyword, request):
    if keyword in self.service_clients:
      return self.service_clients[keyword].call(request)
      pass
    else:
      rospy.logerr("call_service: keyword "+keyword+" not present")
      pass
    pass
  def add_service_server(self, topic, type, keyword,  cb):
    if keyword in self.service_servers:
      rospy.logerr("add_service_server: keyword "+keyword+" already used")
      return
      pass
    self.service_servers[keyword] = ServiceServer(topic, type, cb)
    pass
  def remove_service_server(self, keyword):
    if keyword in self.service_servers:
      self.service_servers[keyword].unregister()
      del self.service_servers[keyword]
      pass
    else:
      rospy.logerr("remove_service_server: keyword "+keyword+" not present")
      pass
    pass
  def add_timer(self, duration, cb, keyword, oneshot = False, autostart = True):
    if keyword in self.timers:
      rospy.logerr("add_timer: keyword "+keyword+" already used")
      return
      pass
    self.timers[keyword] = Timer(duration, cb, oneshot, autostart)
    pass
  def start_timer(self, keyword):
    if keyword in self.timers:
      self.timers[keyword].start()
      pass
    else:
      rospy.logerr("start_timer: keyword "+keyword+" not present")
      pass
    pass
  def stop_timer(self, keyword):
    if keyword in self.timers:
      self.timers[keyword].stop()
      pass
    else:
      rospy.logerr("stop_timer: keyword "+keyword+" not present")
      pass
    pass
  def remove_timer(self, keyword):
    if keyword in self.timers:
      self.timers[keyword].stop()
      del self.timers[keyword]
      pass
    else:
      rospy.logerr("remove_timer: keyword "+keyword+" not present")
      pass
    pass
  def get_timer(self, keyword):
    if keyword in self.timers:
      return self.timers[keyword]
      pass
    rospy.logerr("get_timer: keyword "+keyword+" not present")
    pass
  pass

class NodeConfigure:
  def __init__(self):
    self.initialized_         = False
    self.global_path_         = '/'
    self.local_param_path_    = ''
    self.node_name_           = ''
    self.communication_configure = DefaultCommunicationConfigure(self)
    pass

  def client_side(self):
    self.communication_configure.client_side()

  def get_param_safe(self, param):
    if rospy.has_param(param):
      return rospy.get_param(param)
      pass
    else:
      rospy.logerr('get_param_safe: param '+param+' not set')
      rospy.signal_shutdown('get_param_safe: param '+param+' not set')
      pass
    pass
  def get_param_safe_with_default_value(self, param, default_value):
    if rospy.has_param(param):
      return rospy.get_param(param)
      pass
    else:
      rospy.logwarn('get_param_safe: param '+param+' not set, using default value '+ str(default_value))
      return default_value
      pass
    pass

  def configure_verbose_level(self):
    rospy.logwarn('TODO: verbose level: '+self.get_param_safe_with_default_value(self.verbose_level_path()+self.node_name_, ''))
    pass

  def initialize(self):
    self.node_name_           = rospy.get_name()
    self.global_path_         = self.get_param_safe('/global_path')
    self.local_param_path_    = self.get_param_safe_with_default_value(self.global_path_+'local_path'+self.node_name_, '')

    self.configure_verbose_level()

    if self.has_local_param('delay_start'):
      rospy.sleep( self.get_local_param('delay_start') )
      pass

    if self.has_local_param('client'):
      self.communication_configure = ClientCommunicationConfigure(self)
    pass
  def get_global_param(self,param, default_value = None):
    if default_value is None:
      return self.get_param_safe(self.global_param_path()+param)
      pass
    else:
      return self.get_param_safe_with_default_value(self.global_param_path()+param, default_value)
      pass
    pass
  def get_local_param(self,param, default_value = None):
    if default_value is None:
      return self.get_param_safe(self.local_param_path()+param)
      pass
    else:
      return self.get_param_safe_with_default_value(self.local_param_path()+param, default_value)
      pass
    pass
  def get_named_action_topic(self,name):
    return self.get_param_safe(self.action_name_path()+name)
    pass
  def get_named_topic(self,name):
    return self.communication_configure.get_named_topic(name)
    pass
  def _get_named_topic(self, name):
    return self.get_param_safe(self.topic_name_path()+name)
  def get_named_node(self,name):
    return self.get_param_safe(self.node_name_path()+name)
    pass
  def get_named_service(self,name):
    return self.get_param_safe(self.service_name_path()+name)
    pass
  def get_named_namespace(self,name):
    return self.get_param_safe(self.namespace_name_path()+name)
    pass
  def has_global_param(self,param):
    return self.has_param(self.global_param_path()+param)
    pass
  def has_local_param(self,param):
    return self.has_param(self.local_param_path()+param)
    pass
  def get_named_namespace_list(self):
    return list(self.get_param_safe(self.namespace_name_path()))
    pass
  def get_named_action_topic_list(self):
    return list(self.get_param_safe(self.action_name_path()))
    pass
  def get_named_topic_list(self):
    return list(self.get_param_safe(self.topic_name_path()))
    pass
  def get_named_service_list(self):
    return list(self.get_param_safe(self.service_name_path()))
    pass
  def get_named_node_list(self):
    return list(self.get_param_safe(self.node_name_path()))
    pass
  def get_named_namespace_list(self):
    return list(self.get_param_safe(self.namespace_name_path()))
    pass
  def has_param(self,param):
    return rospy.has_param(param)
    pass
  def action_name_path(self):
    return self.global_path_+"action_name/"
    pass
  def topic_name_path(self):
    return self.global_path_+"topic_name/"
    pass
  def service_name_path(self):
    return self.global_path_+"service_name/"
    pass
  def node_name_path(self):
    return self.global_path_+"node_name/"
    pass
  def namespace_name_path(self):
    return self.global_path_+"namespace/"
    pass
  def global_param_path(self):
    return self.global_path_+"param/"
    pass
  def local_param_path(self):
    return self.local_param_path_
    pass
  def verbose_level_path(self):
    return self.global_path_+"verbose_level/"
    pass

pass


class DefaultCommunicationConfigure(object):

  def __init__(self, node_configure):
    self.node_configure = node_configure

  def client_side(self):
    return False

  def get_named_topic(self, name):
    return self.node_configure._get_named_topic(name)

class ClientCommunicationConfigure(DefaultCommunicationConfigure):

  def client_side(self):
    return True

  def get_named_topic(self, name):
    default = super(ClientCommunicationConfigure, self).get_named_topic(name)

    return "/"+self.node_configure.get_global_param("bridge_namespace")+default