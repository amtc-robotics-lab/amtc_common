/**
 * @file Communications.h
 *
 * @author Isao Parra
 *
 * @date 2014/04/30 (creation)
 */
#ifndef COMMUNICATIONS_H_
#define COMMUNICATIONS_H_

#include <ros/ros.h>
#include <ros/timer.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/any.hpp>
#include <amtc_common/Debug.h>
#include <vector>
#include <map>

#include <amtc_common/NodeConfigure.h>
#include <amtc_common_msgs/CommunicationStatus.h>
#include <boost/thread/mutex.hpp>


namespace amtc
{
class NodeConfigure;

typedef int SubscriberID;
typedef int PublisherID;
typedef int ServiceServerID;
typedef int ServiceClientID;
typedef int TimerID;
typedef int SimpleActionServerID;
typedef int SimpleActionClientID;

const static SubscriberID         SubscriberID_invalid_ID    = 0;
const static PublisherID          PublisherID_invalid_ID     = 0;
const static ServiceServerID      ServiceServerID_invalid_ID = 0;
const static ServiceClientID      ServiceClientID_invalid_ID = 0;
const static TimerID              TimerID_invalid_ID         = 0;
const static SimpleActionServerID SimpleActionServerID_invalid_ID = 0;
const static SimpleActionClientID SimpleActionClientID_invalid_ID = 0;
typedef std::string KeyWord;
typedef std::string Topic;
typedef boost::any SimpleActionServer;
typedef boost::any SimpleActionClient;

class Communications
{
  friend class Management;

  ros::NodeHandle*  n_;
  boost::mutex      mutex_;

  /*
   * Subscribers
	 */
  std::set<SubscriberID>                  ros_subscribers_ids_;
  std::map<SubscriberID, Topic>           ros_subscriber_id_topic_;
  std::map<KeyWord, SubscriberID>         ros_subscriber_keyword_id_;
  std::map<SubscriberID, ros::Subscriber> ros_subscribers_;

	/*
	 * Publishers
	 */
  std::set<PublisherID>                   ros_publishers_ids_;
  std::map<PublisherID, Topic>            ros_publisher_id_topic_;
  std::map<KeyWord, PublisherID>          ros_publisher_keyword_id_;
  std::map<PublisherID, ros::Publisher>   ros_publishers_;

	/*
	 * Timers
	 */
  std::set<TimerID>             timers_ids_;
  std::map<KeyWord, TimerID>    timer_keyword_id_;
  std::map<TimerID, ros::Timer> timers_;
  std::map<TimerID, double>     timers_period_;
  std::map<TimerID, bool>       timers_oneshot_;
  std::map<TimerID, bool>       timers_autostart_;

  /*
   * ServiceServer
   */
  std::set<ServiceServerID>                     ros_service_servers_ids_;
  std::map<ServiceServerID, Topic>              ros_service_server_id_topic_;
  std::map<KeyWord, ServiceServerID>            ros_service_server_keyword_id_;
  std::map<ServiceServerID, ros::ServiceServer> ros_service_servers_;
  /*
   * ServiceClient
   */
  std::set<ServiceClientID>                     ros_service_clients_ids_;
  std::map<ServiceClientID, Topic>              ros_service_client_id_topic_;
  std::map<KeyWord, ServiceClientID>            ros_service_client_keyword_id_;
  std::map<ServiceClientID, ros::ServiceClient> ros_service_clients_;

  /*
   * SimpleActionServer
   */
  //std::set<SimpleActionServerID>                      ros_simple_action_servers_ids_;
  //std::map<SimpleActionServerID, Topic>               ros_simple_action_server_id_topic_;
  //std::map<KeyWord, SimpleActionServerID>             ros_simple_action_server_keyword_id_;
  //std::map<SimpleActionServerID, SimpleActionServer>  ros_simple_action_servers_;

  /*
   * SimpleActionClient
   */
  //std::set<SimpleActionClientID>                      ros_simple_action_clients_ids_;
  //std::map<SimpleActionClientID, Topic>               ros_simple_action_client_id_topic_;
  //std::map<KeyWord, SimpleActionClientID>             ros_simple_action_client_keyword_id_;
  //std::map<SimpleActionClientID, SimpleActionServer>  ros_simple_action_clients_;

	/*
	 * Spin related
	 */
	float spin_time_;
	ros::Rate* rate_ptr_;

  /*
   * Node List related
   */
  std::vector<std::string> node_list_;

  /*
   * NodeConfigure related
   */
  NodeConfigure* node_configure_ptr_;

public:

	Communications(ros::NodeHandle& n);
	virtual ~Communications();

  void spin();
	void spinLoop();
	void set_spin_time(float spin_time);

  void start_node_monitor(float period);
  void stop_node_monitor();
  void check_nodes_timer_callback(const ros::TimerEvent& e);
  void start_communications_status(float period);
  void stop_communications_status();
  void communications_status_callback(const ros::TimerEvent& e);
  bool is_node_running(std::string node_name);

  template<class M>	SubscriberID add_ros_subscriber(const	std::string& topic, void(*fp)(M), const KeyWord& keyword = "", uint32_t queue_size = 100)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_subscriber_keyword_id_.count(keyword) != 0)
		{
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
			return ros_subscriber_keyword_id_[keyword];
		}

		SubscriberID id	= get_new_id(ros_subscribers_ids_);
		ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID, Topic>(id, topic));

    if(keyword != "")
		{
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord, SubscriberID>(keyword, id));
		}

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe(topic, queue_size, fp)));
		return id;
  }

  template<class M>	SubscriberID add_ros_subscriber(const	std::string& topic, void(*fp)(const ros::MessageEvent<M>&), const KeyWord& keyword = "", uint32_t queue_size = 100)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_subscriber_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
      return ros_subscriber_keyword_id_[keyword];
    }

    SubscriberID id	= get_new_id(ros_subscribers_ids_);
    ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID, Topic>(id, topic));

    if(keyword != "")
    {
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord, SubscriberID>(keyword, id));
    }

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe(topic, queue_size, fp)));
    return id;
  }

  template<class M> SubscriberID add_ros_subscriber(const std::string& topic, boost::function<void(M)> fp, const KeyWord& keyword = "", uint32_t queue_size = 100)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_subscriber_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
      return ros_subscriber_keyword_id_[keyword];
    }

    SubscriberID id = get_new_id(ros_subscribers_ids_);
    ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID,Topic>(id, topic));

    if(keyword != "")
    {
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord,SubscriberID>(keyword, id));
    }

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe(topic, queue_size, fp)));

    return id;
  }

  template<class M, class T> SubscriberID add_ros_subscriber(const	std::string& topic, void(T::*fp)(M), T* obj, const KeyWord& keyword = "", uint32_t queue_size = 100)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_subscriber_keyword_id_.count(keyword) != 0)
		{
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
			return ros_subscriber_keyword_id_[keyword];
		}

		SubscriberID id	= get_new_id(ros_subscribers_ids_);
		ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID, Topic>(id, topic));

    if(keyword != "")
		{
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord, SubscriberID>(keyword, id));
		}

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe(topic, queue_size, fp, obj)));

    return id;
  }

  template<typename M> SubscriberID add_ros_subscriber(const std::string& topic, void(*fp)(const typename M::Ptr&, const std::string&), const KeyWord& keyword = "", uint32_t queue_size = 100)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_subscriber_keyword_id_.count(keyword) != 0)
		{
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
			return ros_subscriber_keyword_id_[keyword];
		}

		SubscriberID id	= get_new_id(ros_subscribers_ids_);
		ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID, Topic>(id, topic));

    if(keyword != "")
		{
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord, SubscriberID>(keyword, id));
		}

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe<M>(topic, queue_size, (boost::function<void(const ros::MessageEvent<M>&)>) boost::bind(&Communications::subscriber_with_publisher_id_cb<M>,this,_1, fp)   )));

    return id;
  }

  template<typename M> SubscriberID add_ros_subscriber(const	std::string& topic, boost::function<void(const typename M::Ptr&,const std::string&)> fp, const KeyWord& keyword="", uint32_t queue_size=100)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_subscriber_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_subscriber: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_subscriber_id_topic_[ros_subscriber_keyword_id_[keyword]].c_str());
      return ros_subscriber_keyword_id_[keyword];
    }

    SubscriberID id	= get_new_id(ros_subscribers_ids_);
    ros_subscribers_ids_.insert(id);
    ros_subscriber_id_topic_.insert(std::pair<SubscriberID, Topic>(id, topic));

    if(keyword != "")
    {
      ros_subscriber_keyword_id_.insert(std::pair<KeyWord, SubscriberID>(keyword,id));
    }

    boost::function<void(const ros::MessageEvent<M>&)> fp2 = boost::bind(&Communications::subscriber_with_publisher_id_cb_2<M>, this, _1, fp);

    ros_subscribers_.insert(std::pair<SubscriberID, ros::Subscriber>(id, n_->subscribe<M>(topic, queue_size, fp2)));

    return id;
  }

  template<class M,class T>	SubscriberID add_ros_subscriber(const	std::string& topic, void(T::*fp)(const typename M::Ptr&, const std::string&), T* obj,const KeyWord& keyword = "", uint32_t queue_size = 100)
  {
    boost::function<void(const typename M::Ptr&, const std::string&)> fp2 = boost::bind(fp, obj, _1, _2);

    return add_ros_subscriber<M>(topic, fp2, keyword, queue_size);
  }

  template<class M> PublisherID add_ros_publisher(const std::string& topic, const KeyWord& keyword = "", uint32_t queue_size = 100)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_publisher_keyword_id_.count(keyword) != 0)
		{
      ROS_WARN_NODE("add_ros_publisher: keyword '%s' already registered with topic '%s'", keyword.c_str(), topic.c_str());
			return ros_publisher_keyword_id_[keyword];
		}

		PublisherID id = get_new_id(ros_publishers_ids_);
		ros_publishers_ids_.insert(id);
    ros_publisher_id_topic_.insert(std::pair<PublisherID, Topic>(id, topic));

    if(keyword != "")
		{
      ros_publisher_keyword_id_.insert(std::pair<KeyWord, PublisherID>(keyword, id));
		}

    ros_publishers_.insert(std::pair<PublisherID, ros::Publisher>(id, n_->advertise<M>(topic, queue_size)));

    return id;
	}
  template<class MReq, class MRes> ServiceServerID add_ros_service(const std::string& topic, bool(*srv_func)(MReq, MRes), const KeyWord& keyword = "")
	{
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_service_server_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_keyword: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_service_server_id_topic_[ros_service_server_keyword_id_[keyword]].c_str());
      return ros_service_server_keyword_id_[keyword];
    }
    ServiceServerID id = get_new_id(ros_service_servers_ids_);
    ros_service_servers_ids_.insert(id);
    ros_service_server_id_topic_.insert(std::pair<ServiceServerID,Topic>(id, topic));

    if(keyword != "")
    {
      ros_service_server_keyword_id_.insert(std::pair<KeyWord, ServiceServerID>(keyword, id));
    }

    ros_service_servers_.insert(std::pair<ServiceServerID, ros::ServiceServer>(id,n_->advertiseService(topic, srv_func)));

    return id;
  }
  template<class MReq, class MRes> ServiceServerID add_ros_service(const std::string& topic, boost::function<bool(MReq, MRes)>srv_func, const KeyWord& keyword = "")
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_service_server_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_keyword: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_service_server_id_topic_[ros_service_server_keyword_id_[keyword]].c_str());
      return ros_service_server_keyword_id_[keyword];
    }
    ServiceServerID id = get_new_id(ros_service_servers_ids_);
    ros_service_servers_ids_.insert(id);
    ros_service_server_id_topic_.insert(std::pair<ServiceServerID, Topic>(id, topic));

    if(keyword != "")
    {
      ros_service_server_keyword_id_.insert(std::pair<KeyWord,ServiceServerID>(keyword,id));
    }

    ros_service_servers_.insert(std::pair<ServiceServerID,ros::ServiceServer>(id, n_->advertiseService(topic, srv_func)));
    return id;
  }
  template<class T,class MReq, class MRes> ServiceServerID add_ros_service(const std::string& topic, bool(T::*srv_func)(MReq, MRes), T* obj,const KeyWord& keyword = "")
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_service_server_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_keyword: keyword '%s' already registered with topic '%s'", keyword.c_str(), ros_service_server_id_topic_[ros_service_server_keyword_id_[keyword]].c_str());
      return ros_service_server_keyword_id_[keyword];
    }

    ServiceServerID id = get_new_id(ros_service_servers_ids_);
    ros_service_servers_ids_.insert(id);
    ros_service_server_id_topic_.insert(std::pair<ServiceServerID, Topic>(id,topic));

    if(keyword != "")
    {
      ros_service_server_keyword_id_.insert(std::pair<KeyWord, ServiceServerID>(keyword,id));
    }

    ros_service_servers_.insert(std::pair<ServiceServerID, ros::ServiceServer>(id,n_->advertiseService(topic, srv_func, obj)));
    return id;
  }
  template<class T> ServiceClientID add_ros_service_client(const std::string& topic, const KeyWord& keyword = "", bool wait_for_service = false)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_service_client_keyword_id_.count(keyword) != 0)
    {
      ROS_WARN_NODE("add_ros_service_client: keyword '%s' already registered with topic '%s'", keyword.c_str(), topic.c_str());
      return ros_service_client_keyword_id_[keyword];
    }

    ServiceClientID id = get_new_id(ros_service_clients_ids_);
    ros_service_clients_ids_.insert(id);
    ros_service_client_id_topic_.insert(std::pair<ServiceClientID, Topic>(id, topic));

    if(keyword != "")
    {
      ros_service_client_keyword_id_.insert(std::pair<KeyWord, ServiceClientID>(keyword, id));
    }

    ros_service_clients_.insert(std::pair<ServiceClientID,ros::ServiceClient>(id,n_->serviceClient<T>(topic)));
    if( wait_for_service == true )
    {
      if(keyword != "")
      {
        wait_for_service_existence(keyword);
      }
      else
      {
        wait_for_service_existence(id);
      }
    }
    return id;
  }

  template<class M> void publish(const PublisherID id, const M &message)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_publishers_ids_.count(id))
		{
			ros_publishers_[id].publish(message);
		}
		else
		{
      ROS_WARN_NODE("publish: publisher_id= %d not valid", id);
		}
	}

  template<class M> void publish(const KeyWord& keyword, const M &message)
	{
    boost::mutex::scoped_lock lock(mutex_);
		if(ros_publisher_keyword_id_.count(keyword) == 0)
		{
      ROS_WARN_NODE("publish: keyword '%s' not registered", keyword.c_str());
			return;
		}

		ros_publishers_[ ros_publisher_keyword_id_[keyword] ].publish(message);
  }

  template<class M> bool call_service(const KeyWord& keyword , M &message)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros_service_client_keyword_id_.count(keyword) == 0)
    {
      ROS_WARN_NODE("service_call: keyword '%s' not registered", keyword.c_str());
      return false;
    }
    else if(!ros_service_clients_[ ros_service_client_keyword_id_[keyword] ].call(message))
    {
      ROS_WARN_NODE("call_service: Failed. Exists=%d. ID=%s Resolved name is: %s", ros_service_clients_[ros_service_client_keyword_id_[keyword]].exists(), keyword.c_str(), n_->resolveName(ros_service_client_id_topic_[ros_service_client_keyword_id_[keyword]]).c_str());
      return false;
    }
    return true;
  }

  template <class M> bool call_service(const ServiceClientID id, M &message)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!ros_service_clients_ids_.count(id))
    {
      ROS_WARN_NODE("call_service: service_client_id= %d not valid", id);
      return false;
    }

    else if(!ros_service_clients_[id].call(message))
    {
      ROS_WARN_NODE("call_service: Failed. Exists=%d. Resolved name is: %s", ros_service_clients_[id].exists(), n_->resolveName(ros_service_client_id_topic_[id]).c_str());
      return false;
    }

    return true;
  }
  bool wait_for_service_existence(const SubscriberID id, ros::Duration timeout = ros::Duration(-1));
  bool wait_for_service_existence(const KeyWord& keyword, ros::Duration timeout = ros::Duration(-1));
	bool stop_ros_subscriber(const KeyWord& keyword);
	bool stop_ros_subscriber(const SubscriberID& id);
	bool stop_ros_publisher(const KeyWord& keyword);
	bool stop_ros_publisher(const PublisherID& id);
	bool stop_ros_service_server(const KeyWord& keyword);
	bool stop_ros_service_server(const ServiceServerID& id);
  bool stop_ros_service_client(const KeyWord& keyword);
  bool stop_ros_service_client(const ServiceClientID& id);	

  int get_num_subscribers(const std::string& topic);

  //bool stop_ros_simple_action_server(const KeyWord& keyword);
  //bool stop_ros_simple_action_server(const SimpleActionServerID& id);
  //bool stop_ros_simple_action_client(const KeyWord& keyword);
  //bool stop_ros_simple_action_client(const SimpleActionClientID& id);
  TimerID add_timer(const KeyWord& keyword,ros::Duration period,const ros::TimerCallback & callback, bool oneshot=false, bool autostart=true);

  template<class T>	TimerID add_timer(const KeyWord& keyword, ros::Duration period, void(T::*callback)(const ros::TimerEvent &), T* obj, bool oneshot = false, bool autostart=true)
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

    timers_.insert(std::pair<TimerID,ros::Timer>(id,n_->createTimer(period, callback, obj,oneshot ,autostart)));
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

	void start_timer(const KeyWord& keyword);
	void start_timer(const TimerID& id);
	void stop_timer(const KeyWord& keyword);
	void stop_timer(const TimerID& id);
	void set_period_timer(const KeyWord& keyword,const ros::Duration period);
	void set_period_timer(const TimerID& id,const ros::Duration period);
  bool stop_and_remove_timer(const KeyWord& keyword);
  bool stop_and_remove_timer(const TimerID& id);
	ros::NodeHandle* getNodeHandler();

  TimerID get_timer_id(const KeyWord& keyword);
  SubscriberID get_subscriber_id(const KeyWord& keyword);
  PublisherID get_publisher_id(const KeyWord& keyword);
  ServiceServerID get_service_server_id(const KeyWord& keyword);
  ServiceClientID get_service_client_id(const KeyWord& keyword);

  NodeConfigure& getNodeConfigure();

private:
  template <typename M> void subscriber_with_publisher_id_cb(const ros::MessageEvent<M>& event, void(*fp)(const typename M::Ptr&, const std::string&))
	{
		fp(event.getMessage(), event.getPublisherName());
	}

  template <typename M> void subscriber_with_publisher_id_cb_2(const ros::MessageEvent<M>& event, boost::function<void(const typename M::Ptr&, const std::string&)> fp)
  {
    fp(event.getMessage(), event.getPublisherName());
  }

	int get_new_id(const std::set<int>& set_ids);
	bool check_valid_timer(const KeyWord& keyword);
	bool check_valid_timer(const TimerID& id);
	bool check_valid_publisher(const KeyWord& keyword);
	bool check_valid_publisher(const PublisherID& id);
  bool check_valid_subscriber(const KeyWord& keyword);
  bool check_valid_subscriber(const SubscriberID& id);
  bool check_valid_service_server(const KeyWord& keyword);
  bool check_valid_service_server(const ServiceServerID& id);
  bool check_valid_service_client(const KeyWord& keyword);
  bool check_valid_service_client(const ServiceClientID& id);


};

typedef Communications* CommunicationsPtr;

} /* namespace amtc */


#endif
