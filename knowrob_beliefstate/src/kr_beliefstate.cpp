#include "knowrob_beliefstate/kr_beliefstate.h"

PREDICATE(service_call_mark_dirty_objects, 1)
{
  std::string stuff((char*)A1);
  ROS_INFO("Arg to foreign prolog predicate %s\n", stuff.c_str());
}
