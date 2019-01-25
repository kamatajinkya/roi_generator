//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_NULLPOINTCLOUDDEBUGGER_HPP
#define PROJECT_NULLPOINTCLOUDDEBUGGER_HPP

#include "AbstractPointCloudDebugger.hpp"

namespace roi_generator {
namespace debugUtils {

class NullPointCloudDebugger : public AbstractPointCloudDebugger {
public:
  NullPointCloudDebugger();
  void debug(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) override;

};

}
}

#endif //PROJECT_NULLPOINTCLOUDDEBUGGER_HPP
