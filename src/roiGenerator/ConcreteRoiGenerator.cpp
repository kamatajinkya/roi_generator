//
// Created by ajinkya on 1/24/19.
//

#include <utility>
#include <ros/console.h>
#include "roi_generator/roiGenerator/ConcreteRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator {

ConcreteRoiGenerator::ConcreteRoiGenerator(std::shared_ptr<roiGenerator::AbstractRoiGenerator> roiGenerationAlgo,
                                           std::shared_ptr<floorRemover::AbstractFloorRemover> floorRemover,
                                           std::shared_ptr<workspaceCropper::AbstractWorkspaceCropper> workspaceCropper,
                                           std::shared_ptr<debugUtils::AbstractPointCloudDebugger> debugger)
: mRoiGenerationAlgo(std::move(roiGenerationAlgo))
, mFloorRemover(std::move(floorRemover))
, mWorkspaceCropper(std::move(workspaceCropper))
, mDebugger(debugger)
{
  //Do Nothing
}

Roi ConcreteRoiGenerator::generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr saved_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  *saved_cloud = *cloud;
  mDebugger->debugCloud(*saved_cloud, "Input_Cloud");

  ROS_INFO("Started Cropping Workspace");
  mWorkspaceCropper->crop(saved_cloud);
  mDebugger->debugCloud(*saved_cloud, "Cropped_Cloud");
  ROS_INFO("Done Cropping Workspace");

  ROS_INFO("Started to Remove Floor");
  mFloorRemover->remove(saved_cloud);
  mDebugger->debugCloud(*saved_cloud, "Floor_Subtracted_Cloud");
  ROS_INFO("Done Removing Floor");

  ROS_INFO("Started Selecting ROI");
  auto roi = mRoiGenerationAlgo->generate(saved_cloud);
  mDebugger->debugROI(roi, *saved_cloud, "Roi_Generated");
  ROS_INFO("Done Removing Floor");


  return roi;
}


}
}