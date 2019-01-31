//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP
#define PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP

#include <vector>
#include <string>
#include <initializer_list>
#include "AbstractPointCloudDebugger.hpp"

namespace roi_generator {
namespace debugUtils {

class CompositePointCloudDebugger : public AbstractPointCloudDebugger {
public:
  explicit CompositePointCloudDebugger(const std::initializer_list<std::shared_ptr<AbstractPointCloudDebugger>>& debugger_list);

  void debugCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ::std::string msg) override;

  void debugROI(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) override;

private:
  ::std::vector<::std::shared_ptr<AbstractPointCloudDebugger>> mDebugger_list;

};

}
}

#endif //PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP
