#include <ros/ros.h>
#include "whycon_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whycon", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");

  whycon::WhyConROS whycon_ros(n);
  ros::spin();
}
