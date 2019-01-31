//
// Created by ajinkya on 1/28/19.
//

#include "roi_generator/roiGenerator/SegmentationBasedGeneratorAlgorithm.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <ros/console.h>

#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"
#include "roi_generator/utils/RansacBasedSegmentation.hpp"
#include "roi_generator/utils/utils.hpp"

#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"

using roi_generator::utils::RansacBasedSegmentation;

namespace roi_generator{
namespace roiGenerator {

Roi SegmentationBasedGeneratorAlgorithm::generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Roi roi(0.1,0.1,0.1);
  bool isValid = false;
  RansacBasedSegmentation segmentor(pcl::SacModel::SACMODEL_PLANE, 0.1);
  debugUtils::VisualPointCloudDebugger visualPointCloudDebugger("RansacBasedSegmentation");
  auto segmentList = segmentor.extractSegments(cloud);
  size_t segmentId = 0;
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
    if(!featureExtractor.getOBB(minPointOOB, maxPointOOB, positionOOB, rotationMatrixOOB))
    {
      ROS_ERROR("Could not find OOB");
      continue;
    }

    segmentId++;
    mDebugger->debugCloud(*segmentedCloud, std::string("Segment : ") + std::to_string(segmentId));
    Eigen::Affine3f transformOOB(Eigen::Translation3f(Eigen::Vector3f(positionOOB.x, positionOOB.y, positionOOB.z)));
    transformOOB.linear() = rotationMatrixOOB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr worldCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*cloud, *worldCloud, transformOOB.inverse());
    pcl::transformPoint(maxPointOOB, transformOOB.inverse());
    pcl::transformPoint(minPointOOB, transformOOB.inverse());
    utils::filterCloud(worldCloud, "x", minPointOOB.x, maxPointOOB.x);
    utils::filterCloud(worldCloud, "y", minPointOOB.y, maxPointOOB.y);

    pcl::PointXYZRGB minPoint;
    pcl::PointXYZRGB maxPoint;

    mDebugger->debugCloud(*worldCloud, std::string("Segment Cropped : ") + std::to_string(segmentId));
    pcl::getMinMax3D(*worldCloud, minPoint, maxPoint);
    double threshold = 0.02;
    if((minPoint.z >= minPointOOB.z - threshold || maxPoint.z <= maxPointOOB.z + threshold))
    {
      Eigen::Vector3f centerOfMass;
      featureExtractor.getMassCenter(centerOfMass);
      roi.x() = centerOfMass[0];
      roi.y() = centerOfMass[1];
      roi.z() = centerOfMass[2];
      mDebugger->debugROI(roi, *cloud, std::string("Roi : ") + std::to_string(segmentId));
      if(mRoiValidator->isValid(roi, *segmentedCloud))
      {
        isValid = true;
        break;
      }
    }
  }

  if(!isValid)
  {
    ROS_ERROR("Could Not find ROI");
  }

  return roi;
}

SegmentationBasedGeneratorAlgorithm::SegmentationBasedGeneratorAlgorithm(
  std::shared_ptr<roiValidator::AbstractRoiValidator> roiValidator,
  std::shared_ptr<debugUtils::AbstractPointCloudDebugger> debugger)
: mRoiValidator(roiValidator)
, mDebugger(debugger)
{

}


}
}

