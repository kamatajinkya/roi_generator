//
// Created by ajinkya on 1/24/19.
//

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include "roi_generator/debugUtils/LogPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

LogPointCloudDebugger::LogPointCloudDebugger(std::string fileLocation)
: mStep(0)
{
  mFileLocation = fileLocation + std::string("/");
  boost::filesystem::create_directory(fileLocation);
  //Do Nothing
}

void LogPointCloudDebugger::debugCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) {
  mStep++;
  pcl::io::savePCDFileASCII(mFileLocation + std::to_string(mStep) + std::string("-") + msg + std::string(".pcd"), cloud);
}

void LogPointCloudDebugger::debugROI(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) {
  mStep++;
  pcl::io::savePCDFileASCII(mFileLocation + std::to_string(mStep) + std::string("-") + msg + std::string(".roi.pcd"), cloud);

  std::ofstream file;
  file.open(mFileLocation + std::to_string(mStep) + std::string("-") + msg + std::string(".roi"));
  file << roi.toString() << std::endl;
  file.close();
}


}
}
