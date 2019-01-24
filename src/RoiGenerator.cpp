//
// Created by ajinkya on 1/23/19.
//

#include <roi_generator/RoiGenerator.hpp>
#include <ros/console.h>
#include <pcl/filters/passthrough.h>

namespace roi_generator {

pcl::PointCloud<pcl::PointXYZRGB> RoiGenerator::removeFloor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double floorHeight)
{
  ROS_INFO("Removing Ground");
//  for(auto iter = cloud->begin(); iter < cloud->end(); iter++){
//    if(iter->z < floorHeight)
//    {
//      cloud->erase(iter);
//    }
//  }
  auto ss = cloud->size();
  pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (floorHeight, 1.0);
  pass.filter(filteredCloud);
  auto es = filteredCloud.size();
  return filteredCloud;
  ROS_INFO("Ground Removed");
}

void RoiGenerator::removeFloor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  removeFloor(cloud, mFloorHeight);
}

void RoiGenerator::setFloorHeight(double floorHeight) {
  mFloorHeight = floorHeight;
}

void RoiGenerator::pruneCloudToWorkspace(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Roi workspace) {
  pcl::PassThrough<pcl::PointXYZRGB> pass;
}

} //namespace roi_generator