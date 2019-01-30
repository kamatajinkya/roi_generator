//
// Created by ajinkya on 1/28/19.
//

#include "roi_generator/roiGenerator/SegmentationBasedGenerator.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <ros/console.h>

#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"
#include "roi_generator/utils/RansacBasedSegmentation.hpp"
#include "roi_generator/utils/utils.hpp"

using roi_generator::utils::RansacBasedSegmentation;

namespace roi_generator{
namespace roiGenerator {

Roi SegmentationBasedGenerator::generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Roi roi(0.1,0.1,0.1);
  bool isValid = false;
  RansacBasedSegmentation segmentor(pcl::SacModel::SACMODEL_PLANE, 0.1);

  auto segmentList = segmentor.extractSegments(cloud);
  for(auto& segment:segmentList)
  {
    auto segmentedCloud = segment.first;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> featureExtractor;
    pcl::PointXYZRGB minPointOOB;
    pcl::PointXYZRGB maxPointOOB;
    pcl::PointXYZRGB positionOOB;
    Eigen::Matrix3f rotationMatrixOOB;

    featureExtractor.setInputCloud(segmentedCloud);
    featureExtractor.compute();
    if(featureExtractor.getOBB(minPointOOB, maxPointOOB, positionOOB, rotationMatrixOOB))
    {
      ROS_ERROR("Could not find OOB");
      continue;
    }

    Eigen::Affine3f transformOOB = Eigen::Affine3f::Identity();
    transformOOB.linear() = rotationMatrixOOB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*cloud, *worldCloud, transformOOB);
    pcl::transformPoint(maxPointOOB, transformOOB);
    pcl::transformPoint(minPointOOB, transformOOB);
    utils::filterCloud(worldCloud, "x", minPointOOB.x, maxPointOOB.x);
    utils::filterCloud(worldCloud, "y", minPointOOB.y, maxPointOOB.y);

    pcl::PointXYZRGB minPoint;
    pcl::PointXYZRGB maxPoint;
    pcl::getMinMax3D(*worldCloud, minPoint, maxPoint);
    double threshold = 0.01;
    if(!(minPoint.z < threshold && maxPoint.z > threshold))
    {
      Eigen::Vector3f centerOfMass;
      featureExtractor.getMassCenter(centerOfMass);
      roi.x() = centerOfMass[0];
      roi.y() = centerOfMass[1];
      roi.z() = centerOfMass[2];
      isValid = true;
      break;
    }
  }

  if(!isValid)
  {
    ROS_ERROR("Could Not find ROI");
  }

  return roi;
}

SegmentationBasedGenerator::SegmentationBasedGenerator() {

}


}
}

