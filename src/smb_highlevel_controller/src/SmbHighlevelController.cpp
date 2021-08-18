#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <cmath>
#include <algorithm>

namespace smb_highlevel_controller {
void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double min = msg->range_max;
    for (int i = 0; i < msg->ranges.size(); ++i) {
      if (msg->ranges[i] < min)
        min = msg->ranges[i];
    }
    ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
}
/*bonus task solution */
void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2 &msg){
  ROS_INFO_STREAM_THROTTLE(2.0, "num poins in 3D cloud: " << msg.data.size());
}
bool SmbHighlevelController::readParameters(){
  if (!nodeHandle_.getParam("/smb_highlevel_controller/scan_subscriber_topic_name", scanTopic_)) {
    ROS_ERROR("Could not find topic parameter!");
    return false;
  }
  //ROS_INFO_STREAM("Read topic: " << scanTopic_);
  int queue_size;
  if (!nodeHandle_.getParam("/smb_highlevel_controller/scan_subscriber_queue_size", subscriberQueueSize_)) {
    ROS_ERROR("Could not find queue_size parameter!");
    return false;
  }
  //ROS_INFO_STREAM("Read queue_size: " << subscriberQueueSize_);
  return true;
}

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle):
    nodeHandle_(nodeHandle), subscriberQueueSize_(10), scanTopic_("/scan") {
  if(!readParameters()){
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO("Instantiation");
  scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
        &SmbHighlevelController::scanCallback, this);
  ROS_INFO("Instantiation scan subscriber complete");
  pclSubscriber_ = nodeHandle_.subscribe("/rslidar_points",1, &SmbHighlevelController::pointcloudCallback, this);
  ROS_INFO("Instantiation on subscriber /rslidar_points");
}

SmbHighlevelController::~SmbHighlevelController() {

}

} /* namespace */
