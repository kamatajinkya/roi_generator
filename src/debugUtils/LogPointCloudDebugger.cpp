//
// Created by ajinkya on 1/24/19.
//

#include <pcl/io/pcd_io.h>
#include "roi_generator/debugUtils/LogPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

LogPointCloudDebugger::LogPointCloudDebugger(std::string filePrefix)
: mFilePrefix(filePrefix)
{
  //Do Nothing
}

void LogPointCloudDebugger::debug(pcl::PointCloud<pcl::PointXYZRGB>& cloud, std::string msg) {
  pcl::io::savePCDFileASCII(mFilePrefix + msg, cloud);
}



}
}
