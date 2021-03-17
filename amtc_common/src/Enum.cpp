/**
 * @file Enum.cpp
 *
 * Implements a function that converts a single comma-separated string
 * of enum names into single entries that can be accessed in
 * constant time.
 *
 * @author Thomas Röfer
 */

#include <amtc_common/Debug.h>
#include <amtc_common/Enum.h>
#include <cassert>
#include <cstring>
#include <stdlib.h>
#include <mutex>

static std::recursive_mutex _mutex;

static char* trim(char* pBegin, char* pEnd)
{
  while(*pBegin == ' ')
    ++pBegin;
  while(pEnd > pBegin && pEnd[-1] == ' ')
    --pEnd;
  *pEnd = 0;
  return pBegin;
}

static int parseValue(char* s)
{
  char* p = strstr(s, "<<");
  char* p2 = strstr(s, "0x");
  char* m;

  int radix = p2 ? 16 : 10;

  if(p)
  {
    char* s1 = trim(s, p);
    char* s2 = p + 2;

    int v1 = strtol(s1, &m, radix);
    int v2 = strtol(s2, &m, radix);

    return v1 << v2;
  }
  else
  {
    return strtol(s, &m, radix);
  }
}

void enumInit(char* enums, const char** names, int numOfEnums)
{
  std::lock_guard<std::recursive_mutex> _lock(_mutex);
  char* pEnd = enums - 1;
  int index = 0;
  bool end;
  do
  {
    char* pBegin = pEnd + 1;
    pEnd = strchr(pBegin, ',');
    end = !pEnd;
    if(end)
      pEnd = pBegin + strlen(pBegin);
    char* name = trim(pBegin, pEnd);
    char* p = strchr(name, '=');
    if(p)
    {
      //assert(/*index && */!strcmp(trim(p + 1, pEnd), names[index - 1]));
      name = trim(name, p);
    }
    assert(index < numOfEnums);
    names[index++] = name;
  }
  while(!end);
}

void complexEnumInit(char* enums, std::map<int, std::string>& names)
{
  std::lock_guard<std::recursive_mutex> _lock(_mutex);
  char* pEnd = enums - 1;
  int index = 0;
  bool end;
  do
  {
    char* pBegin = pEnd + 1;
    pEnd = strchr(pBegin, ',');
    end = !pEnd;
    if(end)
      pEnd = pBegin + strlen(pBegin);
    char* name = trim(pBegin, pEnd);
    char* p = strchr(name, '=');

    ROS_ASSERT_MSG(p, "Complex enums must declare all their values");
    assert(p);

    char* v = p + 1;
    name = trim(name, p);
    int value = parseValue(v);

    //assert(index < numOfEnums);
    names[value] = std::string(name);
  }
  while(!end);
}
