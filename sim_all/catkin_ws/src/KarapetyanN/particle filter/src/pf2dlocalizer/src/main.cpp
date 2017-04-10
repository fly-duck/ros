#include <ros/ros.h>

#include "pf2dlocalizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pf2dlocalizer");
  ros::NodeHandle n;

  pf2dlocalizer pf(n);

  ros::spin();

  return 0;
}
