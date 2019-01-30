//
// Created by ajinkya on 1/28/19.
//

#ifndef PROJECT_SEGMENTATIONBASEDGENERATOR_HPP
#define PROJECT_SEGMENTATIONBASEDGENERATOR_HPP

#include "AbstractRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator{

class SegmentationBasedGenerator : public AbstractRoiGenerator {
public:
  SegmentationBasedGenerator();

  Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;

};

}

}





#endif //PROJECT_SEGMENTATIONBASEDGENERATOR_HPP
