#include "pandarGeneral_sdk/hesai_lidar_client.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include <thread>


HesaiLidarClient::HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh) : recording_(false), bag_(nullptr)
{
  lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar", 10);
  packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets", 10);

  control_service_ = nh.advertiseService("start_stop_recording", &HesaiLidarClient::controlCallback, this);

  std::string serverIp;
  int lidarRecvPort;
  int gpsPort;
  double startAngle;
  std::string lidarCorrectionFile; // Get local correction when getting from lidar failed
  std::string lidarType;
  std::string frameId;
  int pclDataType;
  std::string pcapFile;
  std::string dataType;
  std::string multicastIp;
  bool coordinateCorrectionFlag;
  std::string targetFrame;
  std::string fixedFrame;

  nh.getParam("pcap_file", pcapFile);
  nh.getParam("server_ip", serverIp);
  nh.getParam("lidar_recv_port", lidarRecvPort);
  nh.getParam("gps_port", gpsPort);
  nh.getParam("start_angle", startAngle);
  nh.getParam("lidar_correction_file", lidarCorrectionFile);
  nh.getParam("lidar_type", lidarType);
  nh.getParam("frame_id", frameId);
  nh.getParam("pcldata_type", pclDataType);
  nh.getParam("publish_type", m_sPublishType);
  nh.getParam("timestamp_type", m_sTimestampType);
  nh.getParam("data_type", dataType);
  nh.getParam("multicast_ip", multicastIp);
  nh.getParam("coordinate_correction_flag", coordinateCorrectionFlag);
  nh.getParam("target_frame", targetFrame);
  nh.getParam("fixed_frame", fixedFrame);

  ROS_INFO_STREAM("Got server ip " << serverIp);
  ROS_INFO_STREAM("Got correction file " << lidarCorrectionFile);
  if (!pcapFile.empty())
  {
    hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3),
                                static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile,
                                coordinateCorrectionFlag, targetFrame, fixedFrame);
    if (hsdk != NULL)
    {
      std::ifstream fin(lidarCorrectionFile);
      if (fin.is_open())
      {
        std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
        if (ret != 0)
        {
          std::cout << "Load correction file from " << lidarCorrectionFile << " failed" << std::endl;
        }
        else
        {
          std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
        }
      }
      else
      {
        std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
      }
    }
  }
  else if ("rosbag" == dataType)
  {
    hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3),
                                static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType,
                                lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
    if (hsdk != NULL)
    {
      packetSubscriber = node.subscribe("pandar_packets", 10, &HesaiLidarClient::scanCallback, (HesaiLidarClient *)this, ros::TransportHints().tcpNoDelay(true));
    }
  }
  else
  {
    hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort,
                                boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3),
                                boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,
                                m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
  }

  if (hsdk != NULL)
  {
    hsdk->Start();
    // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
  }
  else
  {
    printf("create sdk fail\n");
  }
}

bool HesaiLidarClient::controlCallback(hesai_lidar::StartStop::Request &req,
                                       hesai_lidar::StartStop::Response &res)
{
  if (req.start)
  {
    if (recording_)
    {
      res.success = false;
      res.message = "Recording is already in progress.";
      return true;
    }
    if (req.filename.empty())
    {
      res.success = false;
      res.message = "Filename must be provided when starting recording.";
      return true;
    }
    StartRecording(req.filename);
    res.success = true;
    res.message = "Started recording to: " + current_filename_;
  }
  else
  {
    if (!recording_)
    {
      res.success = false;
      res.message = "No recording is currently active.";
      return true;
    }
    StopRecording();
    res.success = true;
    res.message = "Stopped recording.";
  }
  return true;
}

void HesaiLidarClient::StartRecording(const std::string &bagname) {
  std::lock_guard<std::mutex> lock(bag_mutex); 
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(bagname, rosbag::bagmode::Write); 
    printf("Create bag file: %s.\n", bagname.c_str()); 
  }
  current_filename_ = bagname;
  recording_ = true;
}

void HesaiLidarClient::StopRecording() {
  std::lock_guard<std::mutex> lock(bag_mutex);
  if (bag_) { 
    printf("Waiting to save the bag file.\n"); 
    bag_->close(); 
    printf("Save the bag file %s successfully.\n", current_filename_.c_str()); 
    bag_ = nullptr; 
  }
  current_filename_ = "";
  recording_ = false;
}

void HesaiLidarClient::Stop() {
  std::lock_guard<std::mutex> lock(bag_mutex);
  if (hsdk != NULL)
  {
    hsdk->Stop();
    delete hsdk;
    hsdk = NULL;
  }
  if (bag_) {
    delete bag_;
    bag_ = nullptr;
  }
}

void HesaiLidarClient::lidarCallback(boost::shared_ptr<PPointCloud> cld, double host_stamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
{
  if (m_sPublishType == "both" || m_sPublishType == "points")
  {
    pcl_conversions::toPCL(ros::Time(cld->points[0].timestamp), cld->header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
    std::lock_guard<std::mutex> lock(bag_mutex);
    if (recording_) {
      // std::thread([this, timestamp, output]() { // Capture 'this' to access bag_ inside the thread
      //   bag_->write("/hesai/pandar", ros::Time(host_stamp), output);
      // }).detach();  // detach the thread if you don't need to join it
      bag_->write("/hesai/pandar", ros::Time(host_stamp), output);
    }
#ifdef PRINT_FLAG
    printf("timestamp: %f, point size: %ld.\n", timestamp, cld->points.size());
#endif
  }
  if (m_sPublishType == "both" || m_sPublishType == "raw")
  {
    packetPublisher.publish(scan);
#ifdef PRINT_FLAG
    printf("raw size: %d.\n", scan->packets.size());
#endif
  }
}

void HesaiLidarClient::gpsCallback(int timestamp)
{
#ifdef PRINT_FLAG
  printf("gps: %d\n", timestamp);
#endif
}

void HesaiLidarClient::scanCallback(const hesai_lidar::PandarScanPtr scan)
{
  // printf("pandar_packets topic message received,\n");
  hsdk->PushScanPacket(scan);
}
