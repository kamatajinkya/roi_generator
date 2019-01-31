//
// Created by ajinkya on 1/28/19.
//

#ifndef PROJECT_SEGMENTATIONBASEDGENERATOR_HPP
#define PROJECT_SEGMENTATIONBASEDGENERATOR_HPP

#include "roi_generator/debugUtils/AbstractPointCloudDebugger.hpp"
#include "roi_generator/debugUtils/NullPointCloudDebugger.hpp"
#include "roi_generator/roiValidator/AbstractRoiValidator.hpp"

#include "AbstractRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator{

class SegmentationBasedGeneratorAlgorithm : public AbstractRoiGenerator {
public:
  SegmentationBasedGeneratorAlgorithm(std::shared_ptr<roiValidator::AbstractRoiValidator> roiValidator,
    std::shared_ptr<debugUtils::AbstractPointCloudDebugger> debugger
    = std::make_shared<debugUtils::NullPointCloudDebugger>());

  Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;
private:
  std::shared_ptr<roiValidator::AbstractRoiValidator> mRoiValidator;
  std::shared_ptr<debugUtils::AbstractPointCloudDebugger> mDebugger;

};

}

}





#endif //PROJECT_SEGMENTATIONBASEDGENERATOR_HPP
