
#include <amtc_common/NodeConfigure.h>
#include <amtc_common/Communications.h>


namespace amtc
{
NodeConfigure::NodeConfigure(amtc::Communications* comm_ptr)
{
  comm_ptr_ = comm_ptr;
  is_initialized = false;
}

NodeConfigure::~NodeConfigure()
{
}

void NodeConfigure::initialize()
{
  if ( is_initialized == true )
  {
    return;
  }

  n_                  = comm_ptr_->getNodeHandler();
  node_name_          = ros::this_node::getName();
  global_path_        = get_param<std::string>("/global_path");
  local_param_path_   = get_param<std::string>(global_path_+"local_path"+node_name_, std::string(""));
  is_initialized      = true;

  ROS_ASSERT(local_param_path_!="");

  configure_verbose_level();

  if (has_local_param("delay_start"))
  {
    ros::Duration delay_start_duration(get_local_param<double>("delay_start"));
    delay_start_duration.sleep();
  }
  if (has_local_param("spin_time"))
  {
    comm_ptr_->set_spin_time( get_local_param<double>("spin_time") );
  }
  if (has_local_param("communication_internal_status") && has_local_param("communication_internal_status_period") && get_local_param<bool>("communication_internal_status") == true && get_local_param<double>("communication_internal_status_period") > 0.0 )
  {
    comm_ptr_->start_communications_status( get_local_param<double>("communication_internal_status_period") );
  }
}

void NodeConfigure::configure_verbose_level()
{
  set_verbose_level(get_param<std::string>(verbose_level_path()+node_name_, std::string("")));
}

void NodeConfigure::set_verbose_level(std::string verbose_level)
{
  if( verbose_level == "NONE")
  {
    ros::console::shutdown();
    return;
  }
  else if( verbose_level == "DEBUG" || verbose_level == "ALL")
  {
    ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Debug);
  }
  else if( verbose_level == "INFO")
  {
    ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Info);
  }
  else if( verbose_level == "WARN")
  {
    ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Warn);
  }
  else if( verbose_level == "ERROR")
  {
    ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Error);
  }
  else if( verbose_level == "FATAL")
  {
    ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
  }
  else
  {
    ROS_WARN_NODE("verbose_level not set");
    return;
  }
  ros::console:: notifyLoggerLevelsChanged();
  return;
}

std::string NodeConfigure::get_named_action_topic(std::string name)
{
  return get_param<std::string>(action_name_path()+name);
}
std::string NodeConfigure::get_named_topic(std::string name)
{
  return get_param<std::string>(topic_name_path()+name);
}
std::string NodeConfigure::get_named_node(std::string name)
{
  return get_param<std::string>(node_name_path()+name);
}
std::string NodeConfigure::get_named_service(std::string name)
{
  return get_param<std::string>(service_name_path()+name);
}
std::string NodeConfigure::get_named_namespace(std::string name)
{
  return get_param<std::string>(namespace_name_path()+name);
}
bool NodeConfigure::has_global_param(std::string param)
{
  return has_param(global_param_path()+param);
}
bool NodeConfigure::has_local_param(std::string param)
{
  return has_param(local_param_path()+param);
}
bool NodeConfigure::has_named_node_local_param(std::string node_name, std::string param)
{
  return has_param(get_named_node_local_path(node_name)+param);
}
bool NodeConfigure::get_named_action_topic_list(std::vector<std::string>& list)
{
  return get_named_list<std::string>(action_name_path(), list);
}
bool NodeConfigure::get_named_topic_list(std::vector<std::string>& list)
{
  return get_named_list<std::string>(topic_name_path(), list);
}
bool NodeConfigure::get_named_service_list(std::vector<std::string>& list)
{
  return get_named_list<std::string>(service_name_path(), list);
}
bool NodeConfigure::get_named_node_list(std::vector<std::string>& list)
{
  return get_named_list<std::string>(node_name_path(), list);
}
bool NodeConfigure::get_named_namespace_list(std::vector<std::string>& list)
{
  return get_named_list<std::string>(namespace_name_path(), list);
}
bool NodeConfigure::has_param(std::string param)
{
  return n_->hasParam(param);
}
std::string NodeConfigure::get_named_node_local_path(std::string node_name)
{
  return get_param<std::string>(global_path_+"local_path"+get_named_node(node_name));
}
std::string NodeConfigure::action_name_path()
{
  return global_path_+"action_name/";
}
std::string NodeConfigure::topic_name_path()
{
  return global_path_+"topic_name/";
}
std::string NodeConfigure::service_name_path()
{
  return global_path_+"service_name/";
}
std::string NodeConfigure::node_name_path()
{
  return global_path_+"node_name/";
}
std::string NodeConfigure::namespace_name_path()
{
  return global_path_+"namespace/";
}
std::string NodeConfigure::global_param_path()
{
  return global_path_+"param/";
}
std::string NodeConfigure::local_param_path()
{
  return local_param_path_;
}
std::string NodeConfigure::verbose_level_path()
{
  return global_path_+"verbose_level/";
}
}
