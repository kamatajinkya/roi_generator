//
// Created by ajinkya on 1/24/19.
//

#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <roi_generator/pointCloudReceiver/PCDPointCloudReceiver.hpp>

#include "roi_generator/pointCloudReceiver/PCDPointCloudReceiver.hpp"


namespace roi_generator {
namespace pointCouldReceiver {

PCDPointCloudReceiver::PCDPointCloudReceiver(std::string fileName)
: mFileName(std::move(fileName))
{
  //Do Nothing
}

pcl::PointCloud<pcl::PointXYZRGB> PCDPointCloudReceiver::receive() {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (mFileName, cloud) == -1) //* load the file
  {
    ROS_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  return cloud;
}

}
}
