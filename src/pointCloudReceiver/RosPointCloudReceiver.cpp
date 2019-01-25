//
// Created by ajinkya on 1/24/19.
//

#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "roi_generator/pointCloudReceiver/RosPointCloudReceiver.hpp"

namespace roi_generator {
namespace pointCouldReceiver {

RosPointCloudReceiver::RosPointCloudReceiver(ros::NodeHandle nh,
                                             std::string topicName,
                                             std::string referenceFrameId,
                                             double timeout)
: mNh(nh)
, mTopicName(topicName)
, mReferenceFrameId(referenceFrameId)
, mTimeout(timeout)
{
  //Do Nothing
}

pcl::PointCloud<pcl::PointXYZRGB> RosPointCloudReceiver::receive() {

  ROS_INFO("Recieving Point Cloud");
  auto pointCloud2Message
    = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(mTopicName, mNh, ros::Duration(mTimeout));
  ROS_INFO("Point Cloud Recieved");

  ROS_INFO("Recieving TF");
  ros::Time t0 = ros::Time(0);
  tf::StampedTransform transform;
  tf::TransformListener tfListener(mNh);

  try
  {
    tfListener.waitForTransform(
      mReferenceFrameId, pointCloud2Message->header.frame_id, t0, ros::Duration(mTimeout));

    tfListener.lookupTransform(
      mReferenceFrameId, pointCloud2Message->header.frame_id, t0, transform);
  }
  catch (const tf::ExtrapolationException& ex)
  {
    throw std::runtime_error("TF Out of date");
  }
  ROS_INFO("TF Recieved");

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*pointCloud2Message, cloud);

  Eigen::Isometry3d mapToCloudFrame;
  tf::transformTFToEigen(transform, mapToCloudFrame);

  pcl::transformPointCloud(cloud, cloud, mapToCloudFrame.matrix());

  return cloud;
}

}
}