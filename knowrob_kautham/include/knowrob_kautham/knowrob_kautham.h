#ifndef __KNOWROB_KAUTHAM__

#define __KNOWROB_KAUTHAM__

#include <stdlib.h>
#include <ros/ros.h>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <string>
#include <memory>

#include <iostream>

// Including Kautham stuff
#include <yumik/YumIK.h>
#include <kautham/CheckCollision.h>
#include <kautham/OpenProblem.h>
#include <kautham/SetQuery.h>
#include <kautham/GetPath.h>
#include <kautham/AttachObstacle2RobotLink.h>
#include <kautham/DetachObstacle.h>
#include <kautham/ObsPos.h>
#include <kautham/SetRobotsConfig.h>
#include <kautham/RemoveObstacle.h>
#include <kautham/SetInit.h>
#include <kautham/SetRobControls.h>
#include <kautham/AddObstacle.h>

#endif
