//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ROSPOINTCLOUDRECEIVER_HPP
#define PROJECT_ROSPOINTCLOUDRECEIVER_HPP

#include <ros/ros.h>
#include "AbstractPointCloudReciever.hpp"

namespace roi_generator {
namespace pointCouldReceiver {

class RosPointCloudReceiver : public AbstractPointCloudReceiver{
public:
  RosPointCloudReceiver(ros::NodeHandle nh, std::string topicName, std::string referenceFrameId, double timeout);

  pcl::PointCloud<pcl::PointXYZRGB> receive() override;

private:
  ros::NodeHandle mNh;
  std::string mTopicName;
  std::string mReferenceFrameId;
  double mTimeout;
};

}
}



#endif //PROJECT_ROSPOINTCLOUDRECEIVER_HPP
