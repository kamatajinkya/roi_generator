//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_ROIGENERATOR_HPP
#define PROJECT_ROIGENERATOR_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Roi.hpp"

namespace roi_generator {

class RoiGenerator {
public:

  virtual Roi generate() = 0;

  void pruneCloudToWorkspace(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Roi workspace);

  void removeFloor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZRGB> removeFloor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double floorHeight);
  void setFloorHeight(double floorHeight);
private:
  double mFloorHeight;

};

} //roi_generator
#endif //PROJECT_ROIGENERATOR_HPP
