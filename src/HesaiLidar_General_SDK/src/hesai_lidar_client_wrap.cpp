#include "pandarGeneral_sdk/hesai_lidar_client_wrap.h"
#include "pandarGeneral_sdk/hesai_lidar_client.h"

HesaiLidarClientWrap::HesaiLidarClientWrap(ros::NodeHandle node, ros::NodeHandle nh) : 
    client(std::make_shared<HesaiLidarClient>(node, nh)) {

}

void HesaiLidarClientWrap::StartRecording(const std::string &bagname) {
  client->StartRecording(bagname);
}

void HesaiLidarClientWrap::StopRecording() {
  client->StopRecording();
}
