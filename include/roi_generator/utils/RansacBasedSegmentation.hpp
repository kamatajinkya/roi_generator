//
// Created by ajinkya on 1/29/19.
//

#ifndef PROJECT_RANSACBASEDSEGMENTATION_HPP
#define PROJECT_RANSACBASEDSEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

namespace roi_generator {
namespace utils {

class RansacBasedSegmentation {
public:
  RansacBasedSegmentation(pcl::SacModel model, double terminationPercent);

  std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients>>
    extractSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
  double mTerminationPercent;
  pcl::SacModel mModel;
};

}
}

#endif //PROJECT_RANSACBASEDSEGMENTATION_HPP
