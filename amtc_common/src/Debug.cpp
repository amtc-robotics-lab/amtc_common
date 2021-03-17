/**
 * @file Debug.cpp
 *
 * Simple debug macros to write cleaner code
 *
 * @author Kenzo Lobos
 *
 * @date 2018/06/25
 */

#include <amtc_common/Debug.h>

int _debugger_present = -1;

void _sigtrap_handler(int)
{
  ROS_ERROR_NODE("Debugger not found. Ignoring software breakpoints");

  _debugger_present = 0;
  signal(SIGTRAP, SIG_DFL);
}
