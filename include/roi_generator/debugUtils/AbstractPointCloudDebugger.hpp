//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ABSTRACTPOINTCLOUDDEBUGGER_HPP
#define PROJECT_ABSTRACTPOINTCLOUDDEBUGGER_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace roi_generator{
namespace debugUtils{

class AbstractPointCloudDebugger {
public:
  virtual void debug(pcl::PointCloud<pcl::PointXYZRGB>& cloud, std::string msg) = 0;
};

}
}



#endif //PROJECT_ABSTRACTPOINTCLOUDDEBUGGER_HPP
