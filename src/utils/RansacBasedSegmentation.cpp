//
// Created by ajinkya on 1/29/19.
//
#include "roi_generator/utils/RansacBasedSegmentation.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "roi_generator/debugUtils/VisualPointCloudDebugger.hpp"

namespace roi_generator {
namespace utils {


RansacBasedSegmentation::RansacBasedSegmentation(pcl::SacModel model, double terminationPercent)
: mModel(model)
, mTerminationPercent(terminationPercent)
{
  // Do Nothing
}

std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients>>
  RansacBasedSegmentation::extractSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

  std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients>> segmentList;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (mModel);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);


  debugUtils::VisualPointCloudDebugger visualPointCloudDebugger("RansacBasedSegmentation");

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  int i = 0;
  unsigned long num_points = (int) cloud->points.size();
  pcl::PointCloud<pcl::PointXYZRGB> filteredCloud(*cloud);
  auto filteredCloudPtr = filteredCloud.makeShared();

  while(filteredCloudPtr->points.size() > mTerminationPercent*num_points)
  {
    i++;
    seg.setInputCloud (filteredCloudPtr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.empty())
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      break;
    }

    extract.setInputCloud(filteredCloudPtr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.filter(*segmentedCloud);
    segmentList.emplace_back(std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                          pcl::ModelCoefficients>(segmentedCloud,*coefficients));

    visualPointCloudDebugger.debug(*segmentedCloud, std::string("Segment : ") + std::to_string(i));

    extract.setNegative(true);
    extract.filter(*filteredCloudPtr);

  }


  return segmentList;
}
}
}