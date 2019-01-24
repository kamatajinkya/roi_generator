//
// Created by ajinkya on 1/23/19.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include "roi_generator/CenteroidBasedGenerator.hpp"

int main(int argc, char** argcv)
{
  ros::init(argc, argcv, "roi_generator_node");
  ros::NodeHandle nh;   //TODO: Deal with namespace before deploy/release

  ROS_INFO("Recieving Point Cloud");

  auto pointCloud2Message
    = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
      std::string("/center_realsense/depth_registered/points_throttled"), nh, ros::Duration(5.));
  ROS_INFO("Point Cloud Recieved");

  ROS_INFO("Recieving TF");
  ros::Time t0 = ros::Time(0);
  tf::StampedTransform transform;
  tf::TransformListener tfListener(nh);
  std::string referenceFrameId("map");
  try
  {
    tfListener.waitForTransform(
      referenceFrameId, pointCloud2Message->header.frame_id, t0, ros::Duration(20.0));

    tfListener.lookupTransform(
      referenceFrameId, pointCloud2Message->header.frame_id, t0, transform);
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

  ROS_INFO("Generating ROI");
  roi_generator::CenteroidBasedGenerator roiGenerator(cloud.makeShared(), mapToCloudFrame, 0.01);
  auto roi = roiGenerator.generate();
  ROS_INFO("ROI Generated");

  ROS_INFO("Publishing ROI Marker");
  ros::Publisher roi_pub = nh.advertise<visualization_msgs::Marker>("roi_marker", 1);
  ROS_INFO("ROI position %f %f %f", roi.x(), roi.y(), roi.z());
  while(ros::ok())
  {
    visualization_msgs::Marker roi_marker;
    roi_marker.header.frame_id = "/map";
    roi_marker.header.stamp = ros::Time::now();
    roi_marker.ns = "roi";
    roi_marker.id = 0;
    roi_marker.type = visualization_msgs::Marker::CUBE;
    roi_marker.action = visualization_msgs::Marker::ADD;
    roi_marker.pose.position.x = roi.x();
    roi_marker.pose.position.y = roi.y();
    roi_marker.pose.position.z = roi.z();
    roi_marker.pose.orientation.x = 0.0;
    roi_marker.pose.orientation.y = 0.0;
    roi_marker.pose.orientation.z = 0.0;
    roi_marker.pose.orientation.w = 1.0;
    roi_marker.scale.x = 0.1;
    roi_marker.scale.y = 0.1;
    roi_marker.scale.z = 0.1;
    roi_marker.color.r = 0.0f;
    roi_marker.color.g = 1.0f;
    roi_marker.color.b = 0.0f;
    roi_marker.color.a = 1.0;
    roi_marker.lifetime = ros::Duration();
    roi_pub.publish(roi_marker);

  }


}