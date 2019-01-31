//
// Created by ajinkya on 1/24/19.
//


#include <roi_generator/debugUtils/CompositePointCloudDebugger.hpp>

namespace roi_generator{
namespace debugUtils{

CompositePointCloudDebugger::CompositePointCloudDebugger(
  const std::initializer_list<std::shared_ptr<AbstractPointCloudDebugger>>& debugger_list)
  : mDebugger_list(debugger_list)
{
  //Do Nothing
}

void debugUtils::CompositePointCloudDebugger::debugCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ::std::string msg) {
  for(auto& debugger : mDebugger_list)
    debugger->debugCloud(cloud, msg);
}

void CompositePointCloudDebugger::debugROI(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) {
  for(auto& debugger : mDebugger_list)
    debugger->debugROI(roi, cloud, msg);
}

}
}