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
bool bool_value;
class ObjectTest
{
public:
  ObjectTest(){};
  void internal_cb(const std_msgs::StringConstPtr& str)
  {
    ROS_INFO("internal_cb : %s",str->data.c_str());
  };
  bool internal_service_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
  {
    if (req.data == true)
    {
      resp.message = "internal_service_cb request has true value";
    }
    else
    {
      resp.message = "internal_service_cb request has false value";
    }
    ROS_INFO("internal_service_cb : %s",resp.message.c_str());
    resp.success = true;
    return true;
  };
};

amtc::PublisherID pub01;
amtc::PublisherID pub02;
amtc::PublisherID pub03;
amtc::PublisherID pub04;
amtc::SubscriberID sub01;
amtc::SubscriberID sub02;
amtc::SubscriberID sub03;
amtc::SubscriberID sub04;
amtc::TimerID tim01;
amtc::TimerID tim02;
amtc::TimerID tim03;
amtc::TimerID tim04;

void timer01_cb(const ros::TimerEvent& e)
{
	ROS_INFO("timer01_cb");
	std_msgs::String msg;
	msg.data="timer01_cb: mensaje1";
	comms_ptr->publish(pub01,msg);
	comms_ptr->start_timer(tim02);
}
void timer02_cb(const ros::TimerEvent& e)
{
	ROS_INFO("timer02_cb");
	comms_ptr->stop_timer("timer03");
}
void timer03_cb(const ros::TimerEvent& e)
{
	ROS_INFO("timer03_cb");

}
void timer04_cb(const ros::TimerEvent& e)
{
	ROS_INFO("timer04_cb");
}

void timer05_cb(const ros::TimerEvent& e)
{
  std_srvs::SetBool msg;
  msg.request.data = bool_value;
  bool_value= !bool_value;
  if ( comms_ptr->call_service("service_client",msg) )
  {
    ROS_INFO("timer05_cb: service_client call returned true: %s -> %s", msg.response.success?"true":"false",msg.response.message.c_str());
    comms_ptr->stop_ros_service_client("service_client"); //This part is not necessary in all codes, it will kill the ros service clients
  }
  else
  {
    ROS_INFO("timer05_cb: service_client call returned false");
  }

}

void sub01_cb(const std_msgs::StringConstPtr& str)
{
	ROS_INFO("sub01_cb : %s",str->data.c_str());
	std_msgs::String msg;
	msg.data="sub01_cb: mensaje2";
	comms_ptr->publish("test02",msg);
	comms_ptr->publish("test02a",msg);
}
void sub02_cb(const std_msgs::StringConstPtr& str)
{
	ROS_INFO("sub02_cb : %s",str->data.c_str());
	std_msgs::String msg;
	msg.data="sub01_cb: mensaje2";
	comms_ptr->publish("test03",msg);
	comms_ptr->publish("test03a",msg);
}
void sub03_cb(const std_msgs::StringConstPtr& str)
{
	ROS_INFO("sub03_cb : %s",str->data.c_str());
}

bool service01_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (req.data == true)
  {
    resp.message = "service01_cb request has true value";
  }
  else
  {
    resp.message = "service01_cb request has false value";
  }
  ROS_INFO("service01_cb : %s",resp.message.c_str());
  resp.success = true;
  return true;
}

int main(int argc, char** argv)
{
  //ROS initialization
  ros::init(argc, argv, "test_Communications");
  ros::NodeHandle n;

  comms_ptr = new amtc::Communications(n);
  comms_ptr->set_spin_time(0.01);

  ObjectTest test;

  ROS_INFO("****PUBLISHERS********");
  pub01 = comms_ptr->add_ros_publisher<std_msgs::String>("/test_01","test01",10);
  ROS_INFO("publisher01 /test_01 test01 %d",pub01);
  pub02 = comms_ptr->add_ros_publisher<std_msgs::String>("/test_02","test02",10);
  ROS_INFO("publisher02 /test_02 test02 %d",pub02);
  pub03 = comms_ptr->add_ros_publisher<std_msgs::String>("/test_03","test03",10);
  ROS_INFO("publisher03 /test_03 \"\" %d",pub03);
  pub04 = comms_ptr->add_ros_publisher<std_msgs::String>("/test_04","test04",10);
  ROS_INFO("publisher04 /test_03 test01 %d",pub04);
  ROS_INFO("****SUBSCRIBERS********");
  sub01 = comms_ptr->add_ros_subscriber("/test_01",sub01_cb,"test01");
  sub02 = comms_ptr->add_ros_subscriber("/test_02",sub02_cb,"");
  sub03 = comms_ptr->add_ros_subscriber("/test_04",sub03_cb,"test04");
  comms_ptr->add_ros_subscriber("/test_05",&ObjectTest::internal_cb,&test,"test05");

  ROS_INFO("****SERVICESSERVER********");

  comms_ptr->add_ros_service("/service_test", service01_cb,"service");
  comms_ptr->add_ros_service("/service_internal_test", &ObjectTest::internal_service_cb,&test,"service_internal");

  ROS_INFO("****SERVICESCLIENTS********");
  comms_ptr->add_ros_service_client<std_srvs::SetBool>("/setBool_service","service_client");

  ROS_INFO("****TIMERS********");
  tim01 = comms_ptr->add_timer("timer01",ros::Duration(5),timer01_cb,true,true);
  tim02 = comms_ptr->add_timer("timer02",ros::Duration(10),timer02_cb,false,false);
  tim03 = comms_ptr->add_timer("timer03",ros::Duration(5),timer03_cb,true,true);
  comms_ptr->add_timer("timer05",ros::Duration(5),timer05_cb);

  while(ros::ok())
  {
	  comms_ptr->spin();
  }
}
