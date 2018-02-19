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
#include <kautham/OpenProblem.h>
#include <kautham/CheckCollision.h>
#include <kautham/FindIK.h>

// Including YumIK stuff
#include <yumik/YumIK.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#endif
