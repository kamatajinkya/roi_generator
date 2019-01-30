//
// Created by ajinkya on 1/23/19.
//

#include <memory>

#include <boost/program_options.hpp>
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
#include "roi_generator/roiGenerator/SegmentationBasedGenerator.hpp"
#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"

using roi_generator::debugUtils::RVizRoiVisualizer;
using roi_generator::floorRemover::NaiveFloorRemover;
using roi_generator::roiGenerator::CenteroidBasedGeneratorAlgorithm;
using roi_generator::roiGenerator::ConcreteRoiGenerator;
using roi_generator::workspaceCropper::NaiveWorkspaceCropper;
using roi_generator::pointCouldReceiver::RosPointCloudReceiver;
using roi_generator::debugUtils::VisualPointCloudDebugger;
using roi_generator::roiGenerator::SegmentationBasedGenerator;

namespace po = boost::program_options;

static const char[] program_name = "roi_generatior_node";

int main(int argc, char** argcv)
{
  ros::init(argc, argcv, "program_name");
  ros::NodeHandle nh;   //TODO: Deal with namespace before deploy/release

  ROS_INFO("Setting up Node");
  RosPointCloudReceiver pointCloudReceiver(nh,
                                           std::string("/center_realsense/depth_registered/points_throttled"),
                                           std::string("map"), 5.0);

  roi_generator::Roi workspace(1.30, 0, 0.2, 0.80, 0.80, 0.80);
  auto segmentBasedGenerator = std::make_shared<SegmentationBasedGenerator>();
  auto centroiodBasedGenerator = std::make_shared<CenteroidBasedGeneratorAlgorithm>(-0.01);
  auto naiveFloorRemover = std::make_shared<NaiveFloorRemover>(-0.1);
  auto naiveWorkspaceCropper = std::make_shared<NaiveWorkspaceCropper>(workspace);
  auto visualPointClouddebugger = std::make_shared<VisualPointCloudDebugger>("Test");
  ConcreteRoiGenerator roiGenerator(segmentBasedGenerator, naiveFloorRemover, naiveWorkspaceCropper, visualPointClouddebugger);

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
    visualizer.visualize();
  }


}