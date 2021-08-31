/**
 * @file Communications.cpp
 *
 * @author Isao Parra
 *
 * @date 2014/04/30 (creation)
 * @date 302+/10/25 (review)
 */

#include <amtc_common/Communications.h>

namespace amtc {

Communications::Communications(ros::NodeHandle& n)
  : node_configure_ptr_(new amtc::NodeConfigure(this))
{
	// TODO Auto-generated constructor stub
  n_ = &n;
  spin_time_ = 0.001;
  rate_ptr_ = new ros::Rate(1/spin_time_);
}

Communications::~Communications()
{
	// TODO Auto-generated destructor stub
}

void Communications::set_spin_time(float dt)
{
	spin_time_ = dt;
	rate_ptr_ = new ros::Rate(1/spin_time_);
}

void Communications::spin()
{
	ros::spinOnce();
	rate_ptr_->sleep();
}

void Communications::spinLoop()
{
  while(ros::ok())
  {
    ros::spinOnce();
    rate_ptr_->sleep();
  }
}

void Communications::start_node_monitor(float period)
{
  add_timer("check_nodes_timer", ros::Duration(period), &Communications::check_nodes_timer_callback, this);
}

void Communications::stop_node_monitor()
{
  stop_timer("check_nodes_timer");
}

void Communications::check_nodes_timer_callback(const ros::TimerEvent& e)
{
  node_list_.clear();
  ros::master::getNodes(node_list_);
}

bool Communications::is_node_running(std::string node_name)
{
  for(std::vector<std::string>::iterator it = node_list_.begin(); it != node_list_.end(); ++it)
  {
    if(*it == node_name)
    {
      return true;
    }
  }

  return false;
}

void Communications::start_communications_status(float period)
{
  add_timer("___communications_status", ros::Duration(period), &Communications::communications_status_callback, this);
  add_ros_publisher<amtc_common_msgs::CommunicationStatus>("internal_status", "amtc_communications_status");
}
void Communications::stop_communications_status()
{
  stop_and_remove_timer("___communications_status");
}
void Communications::communications_status_callback(const ros::TimerEvent& e)
{
  amtc_common_msgs::CommunicationStatus comm_status_msg;
  comm_status_msg.header.stamp    = ros::Time::now();
  {

    boost::mutex::scoped_lock lock(mutex_);
    BOOST_FOREACH( auto &data_pair, timer_keyword_id_)
    {
      amtc_common_msgs::TimerInfo timer_info_msg;

      timer_info_msg.id                   = data_pair.second;
      timer_info_msg.keyword              = data_pair.first;
      timer_info_msg.period               = timers_period_[data_pair.second];
      timer_info_msg.isValid              = timers_[data_pair.second].isValid();
      timer_info_msg.hasStarted           = timers_[data_pair.second].hasStarted();
      timer_info_msg.hasPending           = timers_[data_pair.second].hasPending();
      timer_info_msg.oneshot              = timers_oneshot_[data_pair.second];
      timer_info_msg.autostart            = timers_autostart_[data_pair.second];
      timer_info_msg.timer_handle         = (long)&timers_[data_pair.second];

      comm_status_msg.timers.push_back(timer_info_msg);
    }
  }
  publish("amtc_communications_status",comm_status_msg);

}
bool Communications::wait_for_service_existence(const ServiceClientID id, ros::Duration timeout)
{
  ROS_INFO_NODE("wait_for_service_existence: waiting service %u for %f [s]  ", id, timeout.toSec() );
  if( ros_service_clients_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("wait_for_service_existence: id '%u' not registered", id);
    return false;
  }
  bool exist_service = ros_service_clients_[ id ].waitForExistence(timeout);
  if( exist_service == true )
  {
    ROS_INFO_NODE("wait_for_service_existence: service id: %u found", id );
    return true;
  }
  ROS_INFO_NODE("wait_for_service_existence: service id: %u not found", id );
  return false;
}

bool Communications::wait_for_service_existence(const KeyWord& keyword, ros::Duration timeout)
{
  ROS_INFO_NODE("wait_for_service_existence: waiting service %s for %f [s]  ", keyword.c_str(), timeout.toSec() );
  if(ros_service_client_keyword_id_.count(keyword) == 0)
  {
    ROS_WARN_NODE("wait_for_service_existence: keyword '%s' not registered", keyword.c_str());
    return false;
  }
  bool exist_service = ros_service_clients_[ ros_service_client_keyword_id_[keyword] ].waitForExistence(timeout);
  if( exist_service == true )
  {
    ROS_INFO_NODE("wait_for_service_existence: service %s found", keyword.c_str() );
    return true;
  }
  ROS_INFO_NODE("wait_for_service_existence: service %s not found", keyword.c_str() );
  return false;
}

bool Communications::stop_ros_subscriber(const KeyWord& keyword)
{
	if(ros_subscriber_keyword_id_.count(keyword))
	{
		return stop_ros_subscriber(ros_subscriber_keyword_id_[keyword]);
	}
	else
	{
    ROS_WARN_NODE("stop_ros_subscriber: keyword '%s' not registered", keyword.c_str());
		return false;
	}
}

bool Communications::stop_ros_subscriber(const SubscriberID& _id)
{
  boost::mutex::scoped_lock lock(mutex_);
  SubscriberID id = _id;
	if(ros_subscribers_ids_.count(id) == 0)
	{
    ROS_WARN_NODE("stop_ros_subscriber: subscriber_id '%d' not registered", id);
		return false;
	}
	else
	{
	  ros_subscriber_id_topic_.erase(id);
		ros_subscribers_ids_.erase(id);

    for (std::map<KeyWord,SubscriberID>::iterator it=ros_subscriber_keyword_id_.begin(); it!=ros_subscriber_keyword_id_.end();)
    {
      if(it->second == id)
      {
        it = ros_subscriber_keyword_id_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    ros_subscribers_[id].shutdown();
    ros_subscribers_.erase(id);

    return true;
	}
}

bool Communications::stop_ros_publisher(const KeyWord& keyword)
{
	if(ros_publisher_keyword_id_.count(keyword))
	{
		return stop_ros_publisher(ros_publisher_keyword_id_[keyword]);
	}
	else
	{
    ROS_WARN_NODE("stop_ros_publisher: keyword '%s' not registered", keyword.c_str());
		return false;
	}
}

bool Communications::stop_ros_publisher(const PublisherID& _id)
{
  boost::mutex::scoped_lock lock(mutex_);
  PublisherID id = _id;
	if(ros_publishers_ids_.count(id) == 0)
	{
    ROS_WARN_NODE("stop_ros_publisher: publisher_id '%d' not registered", id);
		return false;
	}
	else
	{
		ros_publisher_id_topic_.erase(id);
		ros_publishers_ids_.erase(id);

		for (std::map<KeyWord,PublisherID>::iterator it=ros_publisher_keyword_id_.begin(); it!=ros_publisher_keyword_id_.end();)
		{
			if(it->second == id)
			{
				it = ros_publisher_keyword_id_.erase(it);
			}
      else
      {
        it++;
      }
		}

		ros_publishers_[id].shutdown();
		ros_publishers_.erase(id);
		return true;
	}
}

bool Communications::stop_ros_service_server(const KeyWord& keyword)
{
  if(ros_service_server_keyword_id_.count(keyword))
  {
    return stop_ros_service_server(ros_service_server_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("stop_ros_service_server: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::stop_ros_service_server(const ServiceServerID& _id)
{
  boost::mutex::scoped_lock lock(mutex_);
  ServiceServerID id = _id;
  if(ros_service_servers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("stop_ros_service_server: service_server_id '%d' not registered", id);
    return false;
  }
  else
  {
    ros_service_server_id_topic_.erase(id);
    ros_service_servers_ids_.erase(id);
    for (std::map<KeyWord,ServiceServerID>::iterator it=ros_service_server_keyword_id_.begin(); it!=ros_service_server_keyword_id_.end();)
    {
      if(it->second == id)
      {
        it = ros_service_server_keyword_id_.erase(it);
      }
      else
      {
        ++it;
      }
    }
    ros_service_servers_[id].shutdown();
    ros_service_servers_.erase(id);
    return true;
  }
}

bool Communications::stop_ros_service_client(const KeyWord& keyword)
{
  if(ros_service_client_keyword_id_.count(keyword))
  {
    return stop_ros_service_client(ros_service_client_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("stop_ros_service_client: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}
bool Communications::stop_ros_service_client(const ServiceClientID& _id)
{
  boost::mutex::scoped_lock lock(mutex_);
  ServiceClientID id = _id;
  if(ros_service_clients_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("stop_ros_service_client: service_client_id '%d' not registered", id);
    return false;
  }
  else
  {
    ros_service_client_id_topic_.erase(id);
    ros_service_clients_ids_.erase(id);

    for (std::map<KeyWord,ServiceClientID>::iterator it=ros_service_client_keyword_id_.begin(); it!=ros_service_client_keyword_id_.end(); )
    {
      if(it->second == id)
      {
        it = ros_service_client_keyword_id_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    ros_service_clients_[id].shutdown();
    ros_service_clients_.erase(id);
    return true;
  }
}

/*
bool Communications::stop_ros_simple_action_server(const KeyWord& keyword)
{
  if(ros_simple_action_server_keyword_id_.count(keyword))
  {
    return stop_ros_simple_action_server(ros_simple_action_server_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("stop_ros_simple_action_server: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::stop_ros_simple_action_server(const SimpleActionServerID& id)
{
  if(ros_simple_action_servers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("stop_ros_simple_action_server: simple_action_server_id '%d' not registered", id);
    return false;
  }
  else
  {
    ros_simple_action_server_id_topic_.erase(id);
    ros_simple_action_servers_ids_.erase(id);
    for (std::map<KeyWord,SimpleActionServerID>::iterator it=ros_simple_action_server_keyword_id_.begin(); it!=ros_simple_action_server_keyword_id_.end(); ++it)
    {
      if(it->second == id)
      {
        ros_simple_action_server_keyword_id_.erase(it->first);
      }
    }
    boost::any_cast<actionlib::SimpleActionServer<actionlib_tutorials::Averaging>>(&ros_simple_action_servers_[id]).shutdown();
    ros_simple_action_servers_.erase(id);
    return true;
  }
}

bool Communications::stop_ros_simple_action_client(const KeyWord& keyword)
{
  if(ros_simple_action_client_keyword_id_.count(keyword))
  {
    return stop_ros_simple_action_client(ros_simple_action_client_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("stop_ros_simple_action_client: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::stop_ros_simple_action_client(const SimpleActionClientID& id)
{
  if(ros_simple_action_clients_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("stop_ros_simple_action_client: simple_action_client_id '%d' not registered", id);
    return false;
  }
  else
  {
    ros_simple_action_client_id_topic_.erase(id);
    ros_simple_action_clients_ids_.erase(id);

    for (std::map<KeyWord,SimpleActionClientID>::iterator it=ros_simple_action_client_keyword_id_.begin(); it!=ros_simple_action_client_keyword_id_.end(); ++it)
    {
      if(it->second == id)
      {
        ros_simple_action_client_keyword_id_.erase(it->first);
      }
    }

    ros_simple_action_clients_[id].shutdown();
    ros_simple_action_clients_.erase(id);
    return true;
  }
}
*/

TimerID Communications::add_timer(const KeyWord& keyword,ros::Duration period,const ros::TimerCallback & callback, bool oneshot, bool autostart)
{
  boost::mutex::scoped_lock lock(mutex_);
	if(timer_keyword_id_.count(keyword) != 0)
	{
    ROS_WARN_NODE("add_timer: keyword '%s' already registered ", keyword.c_str());
		return timer_keyword_id_[keyword];
	}

	TimerID id = get_new_id(timers_ids_);
	timers_ids_.insert(id);

  if(keyword != "")
	{
		timer_keyword_id_.insert(std::pair<KeyWord,TimerID>(keyword,id));
	}

	timers_.insert(std::pair<TimerID,ros::Timer>(id,n_->createTimer(period, callback,oneshot,autostart)));
  timers_period_[id]      = period.toSec();
  timers_oneshot_[id]     = oneshot;
  timers_autostart_[id]   = autostart;
  ROS_DEBUG_NODE("add timer: keyword: %s, id: %u, period: %f", keyword.c_str(), id, period.toSec());
  if( autostart == true)
  {
    start_timer( id );
  }
	return id;
}

void Communications::start_timer(const KeyWord& keyword)
{
	if(check_valid_timer(keyword))
	{
		timers_[timer_keyword_id_[keyword]].start();
    ROS_DEBUG_NODE("start timer: keyword: %s, id: %u", keyword.c_str(), timer_keyword_id_[keyword]);
	}
}

void Communications::start_timer(const TimerID& id)
{
	if(check_valid_timer(id))
	{
		timers_[id].start();
    ROS_DEBUG_NODE("start timer: id: %u", id);
	}
}

void Communications::stop_timer(const KeyWord& keyword)
{
  boost::mutex::scoped_lock lock(mutex_);
	if(check_valid_timer(keyword))
	{
		timers_[timer_keyword_id_[keyword]].stop();
    ROS_DEBUG_NODE("stop timer: keyword: %s", keyword.c_str());
	}
}

void Communications::stop_timer(const TimerID& id)
{
  boost::mutex::scoped_lock lock(mutex_);
	if(check_valid_timer(id))
	{
		timers_[id].stop();
    ROS_DEBUG_NODE("start timer: id: %u", id);
	}
}

void Communications::set_period_timer(const KeyWord& keyword,const ros::Duration period)
{
  boost::mutex::scoped_lock lock(mutex_);
	if(check_valid_timer(keyword))
	{
		timers_[timer_keyword_id_[keyword]].setPeriod(period);
    ROS_DEBUG_NODE("set period timer: keyword: %s, period: %f", keyword.c_str(), period.toSec());
	}
}

void Communications::set_period_timer(const TimerID& id,const ros::Duration period)
{
  boost::mutex::scoped_lock lock(mutex_);
	if(check_valid_timer(id))
	{
		timers_[id].setPeriod(period);
    ROS_DEBUG_NODE("set period timer: id: %u, period: %f", id, period.toSec());
	}
}

bool Communications::stop_and_remove_timer(const KeyWord& keyword)
{
  if(timer_keyword_id_.count(keyword))
  {
    return stop_and_remove_timer(timer_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("stop_and_remove_timer: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::stop_and_remove_timer(const TimerID& _id)
{
  boost::mutex::scoped_lock lock(mutex_);
  TimerID id = _id;
  if(timers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("stop_and_remove_timer: timers_id '%d' not registered", id);
    return false;
  }
  else
  {
    timers_ids_.erase(id);
    std::string keyword;
    for (std::map<KeyWord,TimerID>::iterator it=timer_keyword_id_.begin(); it!=timer_keyword_id_.end();)
    {
      if(it->second == id)
      {
        keyword = it->first;
        it = timer_keyword_id_.erase(it);
      }
      else
      {
        ++it;
      }
    }
    timers_[id].stop();
    timers_.erase(id);
    timers_period_.erase(id);  
    timers_oneshot_.erase(id);
    timers_autostart_.erase(id);
    ROS_DEBUG_NODE("stop_and_remove_timer timer: keyword: %s id:%u", keyword.c_str(), id);
    return true;
  }
}


bool Communications::check_valid_timer(const KeyWord& keyword)
{
	if(timer_keyword_id_.count(keyword))
	{
		return check_valid_timer(timer_keyword_id_[keyword]);
	}
	else
	{
    ROS_WARN_NODE("check_valid_timer: keyword '%s' not registered", keyword.c_str());
		return false;
	}
}

bool Communications::check_valid_timer(const TimerID& id)
{
	if(timers_ids_.count(id) == 0)
	{
    ROS_WARN_NODE("check_valid_timer: timer_id '%d' not registered", id);
		return false;
	}
	else
	{
		return true;
	}
}

bool Communications::check_valid_publisher(const KeyWord& keyword)
{
  if(ros_publisher_keyword_id_.count(keyword))
  {
    return check_valid_publisher(ros_publisher_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("check_valid_publisher: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::check_valid_publisher(const PublisherID& id)
{
  if(ros_publishers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("check_valid_publisher: publisher_id '%d' not registered", id);
    return false;
  }
  else
  {
    return true;
  }
}

bool Communications::check_valid_subscriber(const KeyWord& keyword)
{
  if(ros_subscriber_keyword_id_.count(keyword))
  {
    return check_valid_subscriber(ros_subscriber_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("check_valid_subscriber: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::check_valid_subscriber(const SubscriberID& id)
{
  if(ros_subscribers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("check_valid_subscriber: subscriber_id '%d' not registered", id);
    return false;
  }
  else
  {
    return true;
  }
}

bool Communications::check_valid_service_server(const KeyWord& keyword)
{
  if(ros_service_server_keyword_id_.count(keyword))
  {
    return check_valid_service_server(ros_service_server_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("check_valid_service_server: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::check_valid_service_server(const ServiceServerID& id)
{
  if(ros_service_servers_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("check_valid_service_server: service_server_id '%d' not registered", id);
    return false;
  }
  else
  {
    return true;
  }
}

bool Communications::check_valid_service_client(const KeyWord& keyword)
{
  if(ros_service_client_keyword_id_.count(keyword))
  {
    return check_valid_service_client(ros_service_client_keyword_id_[keyword]);
  }
  else
  {
    ROS_WARN_NODE("check_valid_service_client: keyword '%s' not registered", keyword.c_str());
    return false;
  }
}

bool Communications::check_valid_service_client(const ServiceClientID& id)
{
  if(ros_service_clients_ids_.count(id) == 0)
  {
    ROS_WARN_NODE("check_valid_service_client: service_client_id '%d' not registered", id);
    return false;
  }
  else
  {
    return true;
  }
}

TimerID Communications::get_timer_id(const KeyWord& keyword)
{
  if(timer_keyword_id_.count(keyword))
  {
    return timer_keyword_id_[keyword];
  }

  return TimerID_invalid_ID;
}

SubscriberID Communications::get_subscriber_id(const KeyWord& keyword)
{
  if(ros_subscriber_keyword_id_.count(keyword))
  {
    return ros_subscriber_keyword_id_[keyword];
  }

  return SubscriberID_invalid_ID;
}

PublisherID Communications::get_publisher_id(const KeyWord& keyword)
{
  if(ros_publisher_keyword_id_.count(keyword))
  {
    return ros_publisher_keyword_id_[keyword];
  }

  return PublisherID_invalid_ID;
}

ServiceServerID Communications::get_service_server_id(const KeyWord& keyword)
{
  if(ros_service_server_keyword_id_.count(keyword))
  {
    return ros_service_server_keyword_id_[keyword];
  }

  return ServiceServerID_invalid_ID;
}

ServiceClientID Communications::get_service_client_id(const KeyWord& keyword)
{
  if(ros_service_client_keyword_id_.count(keyword))
  {
    return ros_service_client_keyword_id_[keyword];
  }

  return ServiceClientID_invalid_ID;
}

int Communications::get_new_id(const std::set<int>& set_ids)
{
  if(set_ids.size() == 0)
  {
    return 1;
  }
  else
  {
    return (*set_ids.rbegin()) + 1;
  }
}

ros::NodeHandle* Communications::getNodeHandler()
{
	return n_;
}

amtc::NodeConfigure& Communications::getNodeConfigure()
{
  return *node_configure_ptr_;
}


} /* namespace amtc */
