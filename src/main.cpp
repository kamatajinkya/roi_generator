//
// Created by ajinkya on 1/23/19.
//

#include <memory>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "roi_generator/debugUtils/RVizRoiVisualizer.hpp"
#include "roi_generator/roiGenerator/CenteroidBasedGeneratorAlgorithm.hpp"
#include "roi_generator/roiGenerator/ConcreteRoiGenerator.hpp"
#include "roi_generator/floorRemover/NaiveFloorRemover.hpp"
#include "roi_generator/workspaceCropper/NaiveWorkspaceCropper.hpp"
#include "roi_generator/pointCloudReceiver/RosPointCloudReceiver.hpp"

using roi_generator::debugUtils::RVizRoiVisualizer;
using roi_generator::floorRemover::NaiveFloorRemover;
using roi_generator::roiGenerator::CenteroidBasedGeneratorAlgorithm;
using roi_generator::roiGenerator::ConcreteRoiGenerator;
using roi_generator::workspaceCropper::NaiveWorkspaceCropper;
using roi_generator::pointCouldReceiver::RosPointCloudReceiver;

int main(int argc, char** argcv)
{
  ros::init(argc, argcv, "roi_generator_node");
  ros::NodeHandle nh;   //TODO: Deal with namespace before deploy/release

  ROS_INFO("Setting up Node");
  RosPointCloudReceiver pointCloudReceiver(nh,
                                           std::string("/center_realsense/depth_registered/points_throttled"),
                                           std::string("map"), 5.0);

  auto centroiodBasedGenerator = std::make_shared<CenteroidBasedGeneratorAlgorithm>(0.01);
  auto naiveFloorRemover = std::make_shared<NaiveFloorRemover>(-0.05);
  auto naiveWorkspaceCropper = std::make_shared<NaiveWorkspaceCropper>();
  ConcreteRoiGenerator roiGenerator(centroiodBasedGenerator, naiveFloorRemover, naiveWorkspaceCropper);

  RVizRoiVisualizer visualizer(nh);

  ROS_INFO("Done Setting up Node");

  auto cloud = pointCloudReceiver.receive();

  ROS_INFO("Generating ROI");
  auto roi = roiGenerator.generate(cloud.makeShared());
  ROS_INFO("ROI Generated");

  ROS_INFO("Publishing ROI Marker");
  visualizer.addRoi(roi, 1, 0, 0);
  ROS_INFO("ROI position %f %f %f", roi.x(), roi.y(), roi.z());

  while(ros::ok())
  {

  }


}