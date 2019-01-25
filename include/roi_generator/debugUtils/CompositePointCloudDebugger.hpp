//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP
#define PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP

#include <vector>
#include <string>
#include "AbstractPointCloudDebugger.hpp"

namespace roi_generator {
namespace debugUtils {

class CompositePointCloudDebugger : public AbstractPointCloudDebugger {
public:
  CompositePointCloudDebugger(::std::vector<::std::shared_ptr<AbstractPointCloudDebugger>> debuggers);

  void debug(pcl::PointCloud<pcl::PointXYZRGB>& cloud, ::std::string msg) override;

private:
  ::std::vector<::std::shared_ptr<AbstractPointCloudDebugger>> mDebugger_list;

};

}
}

#endif //PROJECT_CONCRETEPOINTCLOUDDEBUGGER_HPP
