//
// Created by ajinkya on 1/30/19.
//

#include "roi_generator/roiValidator/NullRoiValidator.hpp"

namespace roi_generator {
namespace roiValidator {

bool NullRoiValidator::isValid(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  return true;
}

}
}