#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import rospy
import std_msgs.msg
import std_srvs.srv
import math
import amtc_common.Communications as amtc



class TestCommunications():
  def __init__(self):
    rospy.init_node('test_NodeConfigure')
    self.comms = amtc.Communications()
    self.comms.node_configure.initialize()


    aux_list  = self.comms.node_configure.get_named_action_topic_list()
    rospy.logfatal('ACTION TOPIC')
    rospy.logfatal('\t'+str(aux_list))
    rospy.logfatal('\n')

    aux_list  = self.comms.node_configure.get_named_topic_list()
    rospy.logfatal('TOPIC')
    rospy.logfatal('\t'+str(aux_list))
    rospy.logfatal('\n')

    aux_list  = self.comms.node_configure.get_named_service_list()
    rospy.logfatal('SERVICE')
    rospy.logfatal('\t'+str(aux_list))
    rospy.logfatal('\n')

    aux_list  = self.comms.node_configure.get_named_node_list()
    rospy.logfatal('NODE')
    rospy.logfatal('\t'+str(aux_list))
    rospy.logfatal('\n')

    aux_list  = self.comms.node_configure.get_named_namespace_list()
    rospy.logfatal('NAMESPACE')
    rospy.logfatal('\t'+str(aux_list))
    rospy.logfatal('\n')

    rospy.logfatal('GLOBAL_PARAM')
    element_list= ['name_topic', 'timeout', 'sequence']
    for element in element_list:
      rospy.logfatal('\t'+str(element)+' : '+ str(self.comms.node_configure.get_global_param(element)))
      pass
    rospy.logfatal('\n')

    rospy.logfatal('LOCAL_PARAM')
    element_list= ['delay_start', 'spin_time', 'timer_period']
    for element in element_list:
      rospy.logfatal('\t'+str(element)+' : '+ str(self.comms.node_configure.get_local_param(element)))
      pass
    rospy.logfatal('\n')



    pass

  def input_cb(self,msg):
    print ("income: " +msg.data)
    pass

  def timer_cb(self,event):
    self.comms.get_publisher('string_pub').get_msg().data = 'HOLA'
    self.comms.publish('string_pub')
    self.comms.get_service_client('service_client').wait_for_service()
    req = std_srvs.srv.SetBoolRequest()
    req.data = True
    resp = self.comms.call_service('service_client', req)
    self.comms.get_publisher('string_pub').get_msg().data = resp.message
    self.comms.publish('string_pub')

    pass

  def service_server_cb(self,request):
    print ("service_server: "+str(request.data))
    resp = std_srvs.srv.SetBoolResponse()
    resp.message = 'OK service'
    resp.success = True
    return resp
    pass


  def run(self):
    rospy.spin()
    pass

pass


if __name__ == '__main__':
    node = TestCommunications()
    node.run()


