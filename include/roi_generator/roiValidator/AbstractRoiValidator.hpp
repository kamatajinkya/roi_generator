//
// Created by ajinkya on 1/30/19.
//

#ifndef PROJECT_ABSTRACTROIVALIDATOR_HPP
#define PROJECT_ABSTRACTROIVALIDATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "roi_generator/Roi.hpp"

namespace roi_generator{
namespace roiValidator{

class AbstractRoiValidator{
public:
  virtual bool isValid(Roi& roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud) = 0;
};

}
}

#endif //PROJECT_ABSTRACTROIVALIDATOR_HPP
