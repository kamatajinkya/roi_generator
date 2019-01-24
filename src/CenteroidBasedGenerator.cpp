//
// Created by ajinkya on 1/23/19.
//

#include "roi_generator/CenteroidBasedGenerator.hpp"
#include <memory>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>

namespace roi_generator {

CenteroidBasedGenerator::CenteroidBasedGenerator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                 Eigen::Isometry3d& pointCloudOrigin,
                                                 double threshold)
: mThreshold(threshold)
{
  pcl::transformPointCloud(*cloud, mCloud, pointCloudOrigin.matrix());
}

Roi CenteroidBasedGenerator::generate() {

  auto cloud = removeFloor(mCloud.makeShared(), -0.1);
  Eigen::Vector4d centroidPoint;
  pcl::compute3DCentroid<pcl::PointXYZRGB, double>(cloud, centroidPoint);

  Eigen::Vector3d lineEnd = {centroidPoint[0], centroidPoint[1], centroidPoint[2]};
  double lineNorm = lineEnd.norm();
  double minDistanceFromOrigin = std::numeric_limits<double>::max();

  Roi roi(0.1,0.1,0.1);

  for(auto iter = cloud.begin(); iter < cloud.end(); iter++){
    auto point = Eigen::Vector3d(iter->x, iter->y, iter->z);
    if(lineEnd.cross(point).norm()/lineNorm < mThreshold)
    {
      double distanceOfPointFromOrigin = point.norm();
      if(distanceOfPointFromOrigin < minDistanceFromOrigin)
      {
        roi.x() = iter->x;
        roi.y() = iter->y;
        roi.z() = iter->z;
        minDistanceFromOrigin = distanceOfPointFromOrigin;
      }
    }
  }
  std::cout << roi.x() << "," << roi.y() << "," << roi.z() << std::endl;
  return roi;
}

}