#include "knowrob_beliefstate/kr_beliefstate.h"

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

PREDICATE(service_call_mark_dirty_objects, 1)
{
  std::string stuff((char*)A1);
  std::vector<std::string> tokens;
  tokenize(stuff, ',', tokens);
  int maxK = tokens.size();
  for(int k = 0; k < maxK; k++)
      ROS_INFO("Arg to foreign prolog predicate %s\n", tokens[k].c_str());
  return TRUE;
}

