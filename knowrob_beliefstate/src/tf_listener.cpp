#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <knowrob_beliefstate/GetTF.h>

tf::TransformListener* listener;

bool getTf(knowrob_beliefstate::GetTF::Request &req,
         knowrob_beliefstate::GetTF::Response &res)
{
  ROS_INFO("Got request for transform %s %s.", req.ref_frame.c_str(), req.tg_frame.c_str());
  tf::StampedTransform transform;
  listener->lookupTransform(req.ref_frame.c_str(), req.tg_frame.c_str(), ros::Time(0), transform);
  res.tx = transform.getOrigin().x();
  res.ty = transform.getOrigin().y();
  res.tz = transform.getOrigin().z();
  res.rx = transform.getRotation().x();
  res.ry = transform.getRotation().y();
  res.rz = transform.getRotation().z();
  res.rw = transform.getRotation().w();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "krb_tf_listener");
  ros::NodeHandle n;

  listener = new tf::TransformListener();

  ros::ServiceServer service = n.advertiseService("krb_tf_listener/GetTF", getTf);
  ros::spin();

  return 0;
}
