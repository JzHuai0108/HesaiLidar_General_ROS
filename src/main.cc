#include <ros/ros.h>
#include "pandarGeneral_sdk/hesai_lidar_client_wrap.h"
// #define PRINT_FLAG 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClientWrap pandarClientWrap(node, nh);

  ros::spin();
  return 0;
}
