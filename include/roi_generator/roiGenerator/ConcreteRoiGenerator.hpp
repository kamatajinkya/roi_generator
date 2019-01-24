//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_CONCRETEROIGENERATOR_HPP
#define PROJECT_CONCRETEROIGENERATOR_HPP

#include <memory>

#include "roi_generator/floorRemover/AbstractFloorRemover.hpp"
#include "roi_generator/roiGenerator/AbstractRoiGenerator.hpp"
#include "roi_generator/workspaceCropper/AbstractWorkspaceRemover.hpp"

#include "AbstractRoiGenerator.hpp"

namespace roi_generator {
namespace roiGenerator {

class ConcreteRoiGenerator : public AbstractRoiGenerator{
public:
  ConcreteRoiGenerator(std::shared_ptr<roiGenerator::AbstractRoiGenerator> roiGenerationAlgo,
                       std::shared_ptr<floorRemover::AbstractFloorRemover> floorRemover,
                       std::shared_ptr<workspaceCropper::AbstractWorkspaceCropper> workspaceCropper);

  Roi generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;

private:
  std::shared_ptr<roiGenerator::AbstractRoiGenerator> mRoiGenerationAlgo;
  std::shared_ptr<floorRemover::AbstractFloorRemover> mFloorRemover;
    std::shared_ptr<workspaceCropper::AbstractWorkspaceCropper> mWorkspaceCropper;
};

}
}

#endif //PROJECT_CONCRETEROIGENERATOR_HPP
