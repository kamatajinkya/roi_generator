//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_VISUALPOINTCLOUDDEBUGGER_HPP
#define PROJECT_VISUALPOINTCLOUDDEBUGGER_HPP

#include "roi_generator/debugUtils/AbstractPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

class VisualPointCloudDebugger : public AbstractPointCloudDebugger {
public:
  VisualPointCloudDebugger(std::string prefix);

  void debug(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) override;

private:
  std::string mPrefix;
};


}
}

#endif //PROJECT_VISUALPOINTCLOUDDEBUGGER_HPP
