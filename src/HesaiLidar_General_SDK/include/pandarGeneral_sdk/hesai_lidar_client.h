#ifndef INCLUDE_PANDAR_GENERAL_SDK_HESAI_LIDAR_CLIENT_H_
#define INCLUDE_PANDAR_GENERAL_SDK_HESAI_LIDAR_CLIENT_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "hesai_lidar/PandarScan.h"
#include "hesai_lidar/StartStop.h"
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

#include <mutex>


class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh);  

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan);

  void gpsCallback(int timestamp);

  void scanCallback(const hesai_lidar::PandarScanPtr scan);

  bool controlCallback(hesai_lidar::StartStop::Request &req,
                       hesai_lidar::StartStop::Response &res);

  void StartRecording(const std::string &bagname);
  void StopRecording();
  void Stop();

private:
  ros::Publisher lidarPublisher;
  ros::Publisher packetPublisher;
  PandarGeneralSDK* hsdk;
  std::string m_sPublishType;
  std::string m_sTimestampType;
  ros::Subscriber packetSubscriber;

  ros::ServiceServer control_service_;
  bool recording_;
  std::string current_filename_;

  rosbag::Bag *bag_;
  std::mutex bag_mutex;
};

#endif