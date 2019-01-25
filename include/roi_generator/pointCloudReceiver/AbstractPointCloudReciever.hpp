//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ABSTRACTPOINTCLOUDRECIEVER_HPP
#define PROJECT_ABSTRACTPOINTCLOUDRECIEVER_HPP

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace roi_generator {
namespace pointCouldReceiver {

class AbstractPointCloudReceiver {
public:
  virtual pcl::PointCloud<pcl::PointXYZRGB> receive() = 0;

};

}
}

#endif //PROJECT_ABSTRACTPOINTCLOUDRECIEVER_HPP
