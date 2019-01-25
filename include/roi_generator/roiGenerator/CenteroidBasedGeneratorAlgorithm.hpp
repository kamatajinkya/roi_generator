//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_CENTEROIDBASEDGENERATOR_HPP
#define PROJECT_CENTEROIDBASEDGENERATOR_HPP

#include "CenteroidBasedGeneratorAlgorithm.hpp"
#include "AbstractRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator {

class CenteroidBasedGeneratorAlgorithm : public AbstractRoiGenerator {
public:
  CenteroidBasedGeneratorAlgorithm(double threshold);

  Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;

private:
  double mThreshold;
}
;
}
}
#endif //PROJECT_CENTEROIDBASEDGENERATOR_HPP
