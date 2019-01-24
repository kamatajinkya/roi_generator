//
// Created by ajinkya on 1/24/19.
//

#include "roi_generator/floorRemover/NaiveFloorRemover.hpp"

#include <ros/console.h>
#include <pcl/filters/passthrough.h>


namespace roi_generator {
namespace floorRemover {

NaiveFloorRemover::NaiveFloorRemover(double floorHeight) : mFloorHeight(floorHeight)
{

}

void NaiveFloorRemover::remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  ROS_INFO("Removing Ground");

  pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;
  ::pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(mFloorHeight, 1.0);
  pass.filter(filteredCloud);

  *cloud = filteredCloud;

  ROS_INFO("Ground Removed");

}

}
}