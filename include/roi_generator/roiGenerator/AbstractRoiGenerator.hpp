//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_ABSTRACTROIGENERATOR_HPP
#define PROJECT_ABSTRACTROIGENERATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "roi_generator/Roi.hpp"

namespace roi_generator {
namespace roiGenerator {

class AbstractRoiGenerator{
public:
  virtual Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) = 0;
};

}
}

#endif //PROJECT_ABSTRACTROIGENERATOR_HPP
