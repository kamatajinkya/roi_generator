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
  LogPointCloudDebugger(std::string fileLocation = std::string(""));
  void debugCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) override;

  void debugROI(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string msg) override;

private:
  std::string mFileLocation;
  std::size_t mStep;
};


}
}




#endif //PROJECT_LOGPOINTCLOUDDEBUGGER_HPP
