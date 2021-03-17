/**
 * @file Debug.h
 *
 * Simple debug macros to write cleaner code
 *
 * @author Kenzo Lobos
 *
 * @date 2018/06/25
 */

#pragma once

#include <ros/ros.h>
#include <ros/assert.h>

#include <cstring>
#include <string>
#include <cstdlib>

#include <signal.h>

extern int _debugger_present;
void _sigtrap_handler(int);

#define AMTC_BREAKPOINT \
do \
{ \
  if(-1 == _debugger_present) \
  { \
    ROS_ERROR_NODE("Triggered breakpoint. %s:%d", __FILE__, __LINE__); \
    signal(SIGTRAP, _sigtrap_handler); \
    raise(SIGTRAP); \
    break; \
  } \
  ROS_ERROR_NODE("Ignoring breakpoint. %s:%d", __FILE__, __LINE__); \
} while(0)

#define AMTC_COND_BREAKPOINT(cond) \
  if((cond)) \
  { \
    AMTC_BREAKPOINT; \
  }


#define AMTC_ASSERT(cond) \
  if(!(cond)){ \
    AMTC_BREAKPOINT; \
    ROS_ASSERT((cond)); \
  }

#define AMTC_ASSERT_MSG(cond, msg) \
  AMTC_COND_BREAKPOINT(!(cond)); \
  ROS_ASSERT_MSG((cond), (msg));

#define _LOAD_PARAM_FAST(nh, var) \
  if(!((nh).hasParam(parameter))) \
  { \
    std::string parameterString = #var; \
    ROS_WARN_NODE("Parameter %s not found (Resolved name is %s). There is no default value", parameterString.c_str(), (nh).resolveName(parameterString).c_str()); \
  } \
  else \
  { \
    std::string parameterString = #var; \
    (nh).param(parameterString, var) \
  }

#define _LOAD_PARAM_WITH_DEFAULT(nh, parameter, var, def) \
  if(!((nh).hasParam(parameter))) \
  { \
    std::string parameterString = parameter; \
    ROS_WARN_NODE("Parameter %s not found (Resolved name is %s). Using default value", parameterString.c_str(), (nh).resolveName(parameterString).c_str()); \
  } \
  (nh).param(parameter, var, def)

#define _LOAD_PARAM_NO_DEFAULT(nh, parameter, var) \
  if(!((nh).hasParam(parameter))) \
  { \
    std::string parameterString = parameter; \
    ROS_ERROR_NODE("Parameter %s not found (Resolved name is %s). Aborting", parameterString.c_str(), (nh).resolveName(parameterString).c_str()); \
    std::abort(); \
  } \
  (nh).getParam(parameter, var)

#define _LOAD_PARAM_IV_2(...) _LOAD_PARAM_FAST(__VA_ARGS__)
#define _LOAD_PARAM_IV_3(...) _LOAD_PARAM_NO_DEFAULT(__VA_ARGS__)
#define _LOAD_PARAM_IV_4(...) _LOAD_PARAM_WITH_DEFAULT(__VA_ARGS__)

#define _NARGS_IMPL(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, N, ...) N /* NARG trick */
#define _NARGS(...) _NARGS_IMPL(__VA_ARGS__, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)

#define _LOAD_PARAM_III(d) _LOAD_PARAM_IV##_##d
#define _LOAD_PARAM_II(d) _LOAD_PARAM_III(d) /* This one is actually a mandatory macro */
#define _LOAD_PARAM_I(...) _LOAD_PARAM_II(_NARGS(__VA_ARGS__)) (__VA_ARGS__)

#define LOAD_PARAM(...) _LOAD_PARAM_I(__VA_ARGS__)

#define ROS_DEBUG_COND_NODE(cond, msg, ...) ROS_DEBUG_COND(cond, "%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_INFO_COND_NODE(cond, msg, ...) ROS_INFO_COND(cond, "%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_WARN_COND_NODE(cond, msg, ...) ROS_WARN_COND(cond, "%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_ERROR_COND_NODE(cond, msg, ...) ROS_ERROR_COND(cond, "%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)

#define ROS_DEBUG_NODE(msg, ...) ROS_DEBUG("%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_INFO_NODE(msg, ...) ROS_INFO("%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_WARN_NODE(msg, ...) ROS_WARN("%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)
#define ROS_ERROR_NODE(msg, ...) ROS_ERROR("%s: " msg, ros::this_node::getName().c_str(), ##__VA_ARGS__)

#define ROS_DEBUG_PLUGFILTER(msg, ...) ROS_DEBUG("%s: " msg, this->getName().c_str(), ##__VA_ARGS__)
#define ROS_INFO_PLUGFILTER(msg, ...) ROS_INFO("%s: " msg, this->getName().c_str(), ##__VA_ARGS__)
#define ROS_WARN_PLUGFILTER(msg, ...) ROS_WARN("%s: " msg, this->getName().c_str(), ##__VA_ARGS__)
#define ROS_ERROR_PLUGFILTER(msg, ...) ROS_ERROR("%s: " msg, this->getName().c_str(), ##__VA_ARGS__)

#define AMTC_UNUSED(x) (void)(x)

#define ROS_ASSERT_IMPLIES(p, q) ROS_ASSERT(!(p) || (q))
#define ROS_ASSERT_EQUIV(p, q) ROS_ASSERT(((p) || (q)) || ((!(p)) || (!(q))))

#define ROS_ASSERT_IMPLIES_MSG(p, q, msg) ROS_ASSERT_MSG(!(p) || (q), msg)
#define ROS_ASSERT_EQUIV_MSG(p, q, msg) ROS_ASSERT_MSG(((p) || (q)) || ((!(p)) || (!(q))), msg)
