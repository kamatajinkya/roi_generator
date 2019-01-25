//
// Created by ajinkya on 1/24/19.
//

#include <thread>
#include <chrono>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

VisualPointCloudDebugger::VisualPointCloudDebugger(std::string prefix)
: mPrefix(prefix)
{
  //Do Nothing
}


void VisualPointCloudDebugger::debug(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg)
{
  auto cloudPtr = cloud.makeShared();
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (mPrefix + msg));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudPtr, rgb, mPrefix + msg);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, (mPrefix + msg));
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
}


}
}
