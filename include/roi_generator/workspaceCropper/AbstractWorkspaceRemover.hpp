//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ABSTRACTWORKSPACEREMOVER_HPP
#define PROJECT_ABSTRACTWORKSPACEREMOVER_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace roi_generator {
namespace workspaceCropper {

class AbstractWorkspaceCropper {
public:
  virtual void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) = 0;
};

}
}

#endif //PROJECT_ABSTRACTWORKSPACEREMOVER_HPP
