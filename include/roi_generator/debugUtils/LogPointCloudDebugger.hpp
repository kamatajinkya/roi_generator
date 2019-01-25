//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_LOGPOINTCLOUDDEBUGGER_HPP
#define PROJECT_LOGPOINTCLOUDDEBUGGER_HPP

#include "AbstractPointCloudDebugger.hpp"

namespace roi_generator{
namespace debugUtils{

class LogPointCloudDebugger : public AbstractPointCloudDebugger {
public:
  LogPointCloudDebugger(std::string filePrefix = std::string(""));
  void debug(pcl::PointCloud<pcl::PointXYZRGB>& cloud, std::string msg) override;
private:
  std::string mFilePrefix;
};


}
}




#endif //PROJECT_LOGPOINTCLOUDDEBUGGER_HPP
