#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import rospy
import std_msgs.msg
import std_srvs.srv
import math
import amtc_common.Communications as amtc



class TestCommunications():
  def __init__(self):
    rospy.init_node('TestCommunications', anonymous=True)
    self.comms = amtc.Communications()

    self.comms.add_publisher('hola_topic', std_msgs.msg.String, 'string_pub')
    self.comms.add_subscriber('hola_topic', std_msgs.msg.String, 'string_sub', self.input_cb)
    self.comms.add_service_server('service', std_srvs.srv.SetBool, 'service_server', self.service_server_cb)
    self.comms.add_service_client('service', std_srvs.srv.SetBool, 'service_client')

    self.comms.add_timer(rospy.Duration(5.0), self.timer_cb, 'timer' )

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


