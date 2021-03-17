
#ifndef NODECONFIGURE_H_
#define NODECONFIGURE_H_

#include <ros/ros.h>
#include <amtc_common/Debug.h>
#include <boost/foreach.hpp>
#include <amtc_common/Communications.h>
namespace amtc
{

class Communications;

class NodeConfigure
{

private:
  ros::NodeHandle*      n_;
  std::string           global_path_;
  std::string           local_param_path_;
  std::string           node_name_;
  amtc::Communications* comm_ptr_;
  bool                  is_initialized;
public:
  NodeConfigure(amtc::Communications* comm_ptr);
  ~NodeConfigure();

  void initialize();

  std::string                   get_named_action_topic(std::string name);
  std::string                   get_named_topic(std::string name);
  std::string                   get_named_service(std::string name);
  std::string                   get_named_node(std::string name);
  std::string                   get_named_namespace(std::string name);
  template<typename M> M        get_global_param(std::string param_name);
  template<typename M> M        get_global_param(std::string param_name,M default_value);
  template<typename M> void     set_global_param(std::string param_name,M value);
  bool                          has_global_param(std::string param_name);
  template<typename M> M        get_local_param(std::string param_name);
  template<typename M> M        get_local_param(std::string param_name, M default_value);
  template<typename M> void     set_local_param(std::string param_name,M value);
  bool                          has_local_param(std::string param_name);
  template<typename M> M        get_named_node_local_param(std::string node_name, std::string param_name);
  template<typename M> M        get_named_node_local_param(std::string node_name, std::string param_name, M default_value);
  template<typename M> void     set_named_node_local_param(std::string node_name, std::string param_name, M value);
  bool                          has_named_node_local_param(std::string node_name, std::string param_name);
  bool                          get_named_action_topic_list(std::vector<std::string>& list);
  bool                          get_named_topic_list(std::vector<std::string>& list);
  bool                          get_named_service_list(std::vector<std::string>& list);
  bool                          get_named_node_list(std::vector<std::string>& list);
  bool                          get_named_namespace_list(std::vector<std::string>& list);

private:
  void                          configure_verbose_level();
  void                          set_verbose_level(std::string verbose_level);
  template<typename M> void     get_list(std::string param, std::map<std::string,M>& map);
  template<typename M> void     map_to_vector(std::map<std::string,M>& map, std::vector<std::string>& list);
  template<typename M> bool     get_named_list(std::string param_name,std::vector<std::string>& list);
  template<typename M> M        get_param(std::string param_name);
  template<typename M> M        get_param(std::string param_name, M default_value);
  template<typename M> void     set_param(std::string param_name, M value);
  bool                          has_param(std::string param_name);
  std::string                   get_named_node_local_path(std::string node_name);

  std::string action_name_path();
  std::string topic_name_path();
  std::string service_name_path();
  std::string node_name_path();
  std::string namespace_name_path();
  std::string global_param_path();
  std::string local_param_path();
  std::string verbose_level_path();
};

template<typename M> void NodeConfigure::get_list(std::string param, std::map<std::string,M>& map)
{
  LOAD_PARAM(*n_,param,map);
  return;
}
template<typename M> bool NodeConfigure::get_named_list(std::string param,std::vector<std::string>& list)
{
  std::map<std::string,M> map;
  get_list(param, map);
  map_to_vector(map, list);
  if (list.size() > 0)
  {
    return true;
  }
  return false;
}
template<typename M> M NodeConfigure::get_param(std::string param)
{
  M value;
  LOAD_PARAM(*n_,param,value);
  return value;
}
template<typename M> M NodeConfigure::get_param(std::string param, M default_value)
{
  M value;
  LOAD_PARAM(*n_,param,value,default_value);
  return value;
}

template<typename M> void NodeConfigure::set_param(std::string param, M value)
{
  n_->setParam(param, value);
}

template<typename M> M NodeConfigure::get_global_param(std::string name)
{
  return get_param<M>(global_param_path()+name);
}
template<typename M> M NodeConfigure::get_global_param(std::string name, M default_value)
{
  return get_param<M>(global_param_path()+name,default_value);
}
template<typename M> void NodeConfigure::set_global_param(std::string name, M value)
{
  return set_param<M>(global_param_path()+name,value);
}
template<typename M> M NodeConfigure::get_local_param(std::string name)
{
  return get_param<M>(local_param_path()+name);
}
template<typename M> M NodeConfigure::get_local_param(std::string name, M default_value)
{
  return get_param<M>(local_param_path()+name,default_value);
}
template<typename M> void NodeConfigure::set_local_param(std::string name, M value)
{
  return set_param<M>(local_param_path()+name,value);
}
template<typename M> M NodeConfigure::get_named_node_local_param(std::string node_name, std::string name)
{
  return get_param<M>(get_named_node_local_path(node_name)+name);
}
template<typename M> M NodeConfigure::get_named_node_local_param(std::string node_name, std::string name, M default_value)
{
  return get_param<M>(get_named_node_local_path(node_name)+name,default_value);
}
template<typename M> void NodeConfigure::set_named_node_local_param(std::string node_name, std::string name, M value)
{
  return set_param<M>(get_named_node_local_path(node_name)+name,value);
}

template<typename M> void NodeConfigure::map_to_vector(std::map< std::string,M >& map, std::vector<std::string>& list)
{
  /*
  BOOST_FOREACH(auto& pair, map)
  {
    list.push_back(pair.first);
  }
  */
}




}

#endif

