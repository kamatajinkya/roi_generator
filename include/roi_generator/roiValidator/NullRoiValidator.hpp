//
// Created by ajinkya on 1/30/19.
//

#ifndef PROJECT_NULLROIVALIDATOR_HPP
#define PROJECT_NULLROIVALIDATOR_HPP

#include "roi_generator/roiValidator/AbstractRoiValidator.hpp"

namespace roi_generator {
namespace roiValidator {

class NullRoiValidator : public AbstractRoiValidator {
public:
  bool isValid(Roi &roi, pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;
};

}
}

#endif //PROJECT_NULLROIVALIDATOR_HPP
