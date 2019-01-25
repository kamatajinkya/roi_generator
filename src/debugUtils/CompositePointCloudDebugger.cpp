//
// Created by ajinkya on 1/24/19.
//


#include <roi_generator/debugUtils/CompositePointCloudDebugger.hpp>

#include "roi_generator/debugUtils/LogPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

debugUtils::CompositePointCloudDebugger::CompositePointCloudDebugger(
  ::std::vector < ::std::shared_ptr < AbstractPointCloudDebugger >> debugger_list)
 : mDebugger_list(debugger_list)
{
  //Do Nothing
}

void debugUtils::CompositePointCloudDebugger::debug(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ::std::string msg) {
  for(auto& debugger : mDebugger_list)
    debugger->debug(cloud, msg);
}
}
}