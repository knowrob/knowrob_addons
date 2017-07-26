#include "knowrob_beliefstate/kr_beliefstate.h"

#include <ros/ros.h>
#include <knowrob_beliefstate/GetTF.h>
#include <knowrob_beliefstate/DirtyObject.h>

#include <stdlib.h>

void tokenize(std::string const& str, char d, std::vector<std::string> &tokens){
    size_t start = str.find_first_not_of(d), end=start;

    while (start != std::string::npos){
        // Find next occurence of delimiter
        end = str.find(d, start);
        // Push back the token found into vector
        tokens.push_back(str.substr(start, end-start));
        // Skip all occurences of the delimiter to find new start
        start = str.find_first_not_of(d, end);
    }
}

void trim_brackets(std::string &what)
{
    int maxK = what.length();
    if(']' == what[maxK - 1])
        what = what.substr(0, maxK - 1);    
    if('[' == what[0])
        what = what.substr(1);
}

knowrob_beliefstate::GetTF srv;
knowrob_beliefstate::DirtyObject dobj;

PREDICATE(call_ros_info, 1)
{
  std::string stuff((char*)A1);
  ROS_INFO("%s\n", stuff.c_str());
  return TRUE;
}

PREDICATE(service_call_mark_dirty_objects, 1)
{
  std::string stuff((char*)A1);

  std::string cmd = "rosservice call /object_state_publisher/mark_dirty_object ";
  cmd = cmd + "\'" + stuff + "\'";

  ROS_INFO("%s\n", cmd.c_str());
  system(cmd.c_str());

  return TRUE;
}

PREDICATE(get_current_tf, 9)
{
  int argc = 1;
  char **argv = (char**)malloc(1*sizeof(char*));
  argv[0] = (char*)malloc(4*sizeof(char));
  argv[0][0] = 'k'; argv[0][1] = 'r'; argv[0][2] = 'b'; argv[0][3] = 0;
  ros::init(argc, argv, "krb");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<knowrob_beliefstate::GetTF>("krb_tf_listener/GetTF");
  std::string refframe((char*)A1);
  std::string tgframe((char*)A2);
  srv.request.ref_frame = refframe.c_str();
  srv.request.tg_frame = tgframe.c_str();
  if (client.call(srv))
  {
      A3 = (double)(srv.response.tx);
      A4 = (double)(srv.response.ty);
      A5 = (double)(srv.response.tz);
      A6 = (double)(srv.response.rx);
      A7 = (double)(srv.response.ry);
      A8 = (double)(srv.response.rz);
      A9 = (double)(srv.response.rw);
      return TRUE;
  }
  else
  {
      ROS_ERROR("Couldn't reach service /krb_tf_listener/GetTF ; are you sure krb_tf_listener is running?");
      return FALSE;
  }
}


