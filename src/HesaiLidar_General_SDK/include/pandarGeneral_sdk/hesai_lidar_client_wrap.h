#ifndef INCLUDE_PANDAR_GENERAL_SDK_HESAI_LIDAR_CLIENT_WRAP_H_
#define INCLUDE_PANDAR_GENERAL_SDK_HESAI_LIDAR_CLIENT_WRAP_H_

#include <memory>
#include <ros/ros.h>

class HesaiLidarClient;

class HesaiLidarClientWrap
{
public:
  HesaiLidarClientWrap(ros::NodeHandle node, ros::NodeHandle nh);

  void StartRecording(const std::string &bagname);

  void StopRecording();

  void Stop();

private:
  std::shared_ptr<HesaiLidarClient> client;
};

#endif
