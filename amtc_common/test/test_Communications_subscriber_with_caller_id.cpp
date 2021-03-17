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

class Foo
{
public:

  Foo(){}

  void sub_cb(const std_msgs::String::Ptr& msg,const std::string& publisher_id)
  {
    ROS_INFO("sub01_cb : %s :%s ",publisher_id.c_str(), msg->data.c_str());
  }
};

void sub01_cb(const std_msgs::String::Ptr& msg,const std::string& publisher_id)
{
  ROS_INFO("sub01_cb : %s :%s ",publisher_id.c_str(), msg->data.c_str());
}

void sub02_cb(const ros::MessageEvent<std_msgs::String>& event)
{
  ROS_INFO("sub02_cb : %s :%s ",event.getPublisherName().c_str(),event.getMessage()->data.c_str());
}

int main(int argc, char** argv)
{
  //ROS initialization
  ros::init(argc, argv, "test_Communications");
  ros::NodeHandle n;

  Foo foo;

  comms_ptr = new amtc::Communications(n);
  comms_ptr->set_spin_time(0.01);

  ROS_INFO("****SUBSCRIBERS********");

  comms_ptr->add_ros_subscriber<std_msgs::String>("/test",sub01_cb);
  comms_ptr->add_ros_subscriber("/test",sub02_cb);
  comms_ptr->add_ros_subscriber<std_msgs::String>("/test", (boost::function<void(const typename std_msgs::String::Ptr&, const std::string&)>) boost::bind(&Foo::sub_cb, &foo, _1, _2));
  comms_ptr->add_ros_subscriber<std_msgs::String, Foo>("/test", &Foo::sub_cb, &foo);

  while(ros::ok())
  {
	  comms_ptr->spin();
  }
}

