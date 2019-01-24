//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ABSTRACTFLOORREMOVER_HPP
#define PROJECT_ABSTRACTFLOORREMOVER_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace roi_generator {
namespace floorRemover {

class AbstractFloorRemover {
public:
  virtual void remove(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) = 0;
};

}
}
#endif //PROJECT_ABSTRACTFLOORREMOVER_HPP
