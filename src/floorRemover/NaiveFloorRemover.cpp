//
// Created by ajinkya on 1/24/19.
//

#include "roi_generator/floorRemover/NaiveFloorRemover.hpp"

#include <ros/console.h>
#include "roi_generator/utils/utils.hpp"

namespace roi_generator {
namespace floorRemover {

NaiveFloorRemover::NaiveFloorRemover(double floorHeight) : mFloorHeight(floorHeight)
{

}

void NaiveFloorRemover::remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  ROS_INFO("Removing Ground");

  utils::filterCloud(cloud, "z", mFloorHeight, 1.0);

  ROS_INFO("Ground Removed");

}

}
}