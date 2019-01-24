//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_CENTEROIDBASEDGENERATOR_HPP
#define PROJECT_CENTEROIDBASEDGENERATOR_HPP

#include "CenteroidBasedGenerator.hpp"
#include "RoiGenerator.hpp"

namespace roi_generator {
class CenteroidBasedGenerator : RoiGenerator {
public:
  CenteroidBasedGenerator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Isometry3d& pointCloudOrigin, double threshold);
  virtual Roi generate();

private:
  pcl::PointCloud<pcl::PointXYZRGB> mCloud;
  double mThreshold;
};

}
#endif //PROJECT_CENTEROIDBASEDGENERATOR_HPP
