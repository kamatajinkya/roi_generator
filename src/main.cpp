//
// Created by ajinkya on 1/23/19.
//

#include <memory>
#include <chrono>
#include <sstream>
#include <ctime>

#include <boost/lexical_cast.hpp>
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
#include "roi_generator/roiGenerator/SegmentationBasedGeneratorAlgorithm.hpp"
#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"
#include "roi_generator/debugUtils/LogPointCloudDebugger.hpp"
#include "roi_generator/debugUtils/CompositePointCloudDebugger.hpp"
#include "roi_generator/roiValidator/NullRoiValidator.hpp"

using roi_generator::debugUtils::RVizRoiVisualizer;
using roi_generator::floorRemover::NaiveFloorRemover;
using roi_generator::roiGenerator::CenteroidBasedGeneratorAlgorithm;
using roi_generator::roiGenerator::ConcreteRoiGenerator;
using roi_generator::workspaceCropper::NaiveWorkspaceCropper;
using roi_generator::pointCouldReceiver::RosPointCloudReceiver;
using roi_generator::debugUtils::VisualPointCloudDebugger;
using roi_generator::roiGenerator::SegmentationBasedGeneratorAlgorithm;
using roi_generator::debugUtils::LogPointCloudDebugger;
using roi_generator::debugUtils::CompositePointCloudDebugger;
using roi_generator::debugUtils::AbstractPointCloudDebugger;
using roi_generator::roiValidator::NullRoiValidator;

namespace po = boost::program_options;

static const char program_name[] = "roi_generatior_node";

int main(int argc, char** argcv)
{
  ros::init(argc, argcv, "program_name");
  ros::NodeHandle nh;   //TODO: Deal with namespace before deploy/release

  ROS_INFO("Setting up Node");

  /* Initializing Point Cloud Reciever*/
  RosPointCloudReceiver pointCloudReceiver(nh,
                                           std::string("/center_realsense/depth_registered/points_throttled"),
                                           std::string("map"), 5.0);

  /* Initializing Loggers and Debuggers */
  std::cout << "Input Log Name" << std::endl;
  std::string logPrefixSuffix;
  std::cin >> logPrefixSuffix;
  auto currentTime = std::time(0);
  std::string logPrefix = boost::lexical_cast<std::string>(std::put_time(std::localtime(&currentTime), "%Y-%m-%d-%H-%M-%S-%Z")) + logPrefixSuffix;
  auto visualPointCloudDebugger = std::make_shared<VisualPointCloudDebugger>(logPrefix);
  auto logPointCloudDebugger = std::make_shared<LogPointCloudDebugger>(std::string("logs/") + logPrefix);
  auto compositeCloudDebugger =
    std::make_shared<CompositePointCloudDebugger>(
      std::initializer_list<std::shared_ptr<AbstractPointCloudDebugger>>{visualPointCloudDebugger, logPointCloudDebugger});

  /* Initializing the algorithm*/
  roi_generator::Roi workspace(1.30, 0, 0.2, 0.80, 0.80, 0.80);
  auto nullROIValidator = std::make_shared<NullRoiValidator>();
  auto segmentBasedGenerator = std::make_shared<SegmentationBasedGeneratorAlgorithm>(nullROIValidator, compositeCloudDebugger);
  auto centroiodBasedGenerator = std::make_shared<CenteroidBasedGeneratorAlgorithm>(-0.01);
  auto naiveFloorRemover = std::make_shared<NaiveFloorRemover>(-0.1);
  auto naiveWorkspaceCropper = std::make_shared<NaiveWorkspaceCropper>(workspace);
  ConcreteRoiGenerator roiGenerator(segmentBasedGenerator, naiveFloorRemover, naiveWorkspaceCropper, compositeCloudDebugger);

  /* Initializing ROS*/
  RVizRoiVisualizer visualizer(nh);
  ROS_INFO("Done Setting up Node");

  /* Receiving Point Cloud */
  auto cloud = pointCloudReceiver.receive();

  /* Generating ROI */
  ROS_INFO("Generating ROI");
  auto roi = roiGenerator.generate(cloud.makeShared());
  ROS_INFO("ROI Generated");

  /* Visualizing ROI*/
  ROS_INFO("Publishing ROI Marker");
  visualizer.addRoi(roi, 1, 0, 0);
  ROS_INFO("ROI position %f %f %f", roi.x(), roi.y(), roi.z());

  while(ros::ok())
  {
    visualizer.visualize();
  }



}