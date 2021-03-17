/*
 * test_Communications.cpp
 *
 *  Created on: May 31, 2014
 *      Author: isao
 */
#include <amtc_common/Communications.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
amtc::Communications* comms_ptr;

void timer_cb(const ros::TimerEvent& e)
{
  ROS_FATAL("TEST");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_NodeConfigure");
  ros::NodeHandle n("~");

  comms_ptr = new amtc::Communications(n);

  comms_ptr->getNodeConfigure().initialize();

  comms_ptr->add_timer("timer",ros::Duration(comms_ptr->getNodeConfigure().get_local_param<double>("timer_period")),timer_cb);

  {
    std::vector<std::string> list;
    comms_ptr->getNodeConfigure().get_named_action_topic_list(list);
    ROS_FATAL("ACTION_TOPIC");
    BOOST_FOREACH(std::string topic, list)
    {
      ROS_FATAL("\t%s : %s", topic.c_str(), comms_ptr->getNodeConfigure().get_named_action_topic(topic).c_str());
    }
  }
  {
    std::vector<std::string> list;
    comms_ptr->getNodeConfigure().get_named_topic_list(list);
    ROS_FATAL("TOPIC");
    BOOST_FOREACH(std::string topic, list)
    {
      ROS_FATAL("\t%s : %s", topic.c_str(), comms_ptr->getNodeConfigure().get_named_topic(topic).c_str());
    }
  }
  {
    std::vector<std::string> list;
    comms_ptr->getNodeConfigure().get_named_service_list(list);
    ROS_FATAL("SERVICE");
    BOOST_FOREACH(std::string element, list)
    {
      ROS_FATAL("\t%s : %s", element.c_str(), comms_ptr->getNodeConfigure().get_named_service(element).c_str());
    }
  }
  {
    std::vector<std::string> list;
    comms_ptr->getNodeConfigure().get_named_node_list(list);
    ROS_FATAL("NODE");
    BOOST_FOREACH(std::string element, list)
    {
      ROS_FATAL("\t%s : %s", element.c_str(), comms_ptr->getNodeConfigure().get_named_node(element).c_str());
    }
  }
  {
    std::vector<std::string> list;
    comms_ptr->getNodeConfigure().get_named_namespace_list(list);
    ROS_FATAL("NAMESPACE");
    BOOST_FOREACH(std::string element, list)
    {
      ROS_FATAL("\t%s : %s", element.c_str(), comms_ptr->getNodeConfigure().get_named_namespace(element).c_str());
    }
  }
  {
    ROS_FATAL("GLOBAL_PARAM");
    std::string element;
    element = "name_topic";
    ROS_FATAL("\t%s : %s", element.c_str(), comms_ptr->getNodeConfigure().get_global_param<std::string>(element).c_str());
    element = "timeout";
    ROS_FATAL("\t%s : %f", element.c_str(), comms_ptr->getNodeConfigure().get_global_param<double>(element));
    element = "sequence";
    std::vector<double> double_list(comms_ptr->getNodeConfigure().get_global_param<std::vector<double>>(element));
    ROS_FATAL("\t%s : ", element.c_str());
    BOOST_FOREACH(double value, double_list)
    {
      ROS_FATAL("\t\t%f", value);
    }
  }
  {
    ROS_FATAL("LOCAL_PARAM");
    std::string element;
    element = "delay_start";
    ROS_FATAL("\t%s : %f", element.c_str(), comms_ptr->getNodeConfigure().get_local_param<double>(element));
    element = "spin_time";
    ROS_FATAL("\t%s : %f", element.c_str(), comms_ptr->getNodeConfigure().get_local_param<double>(element));
    element = "timer_period";
    ROS_FATAL("\t%s : %f", element.c_str(), comms_ptr->getNodeConfigure().get_local_param<double>(element));
  }



  while(ros::ok())
  {
	  comms_ptr->spin();
  }
}
