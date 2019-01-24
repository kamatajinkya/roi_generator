//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_CENTEROIDBASEDGENERATOR_HPP
#define PROJECT_CENTEROIDBASEDGENERATOR_HPP

#include "CenteroidBasedGenerator.hpp"
#include "AbstractRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator {

class CenteroidBasedGenerator : public AbstractRoiGenerator {
public:
  CenteroidBasedGenerator(double threshold);

  Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;

private:
  double mThreshold;
}
;
}
}
#endif //PROJECT_CENTEROIDBASEDGENERATOR_HPP
