//
// Created by ajinkya on 1/24/19.
//

#include "roi_generator/workspaceCropper/NaiveWorkspaceCropper.hpp"

#include <ros/console.h>
#include <roi_generator/workspaceCropper/NaiveWorkspaceCropper.hpp>

#include "roi_generator/utils/utils.hpp"

namespace roi_generator {
namespace workspaceCropper {

NaiveWorkspaceCropper::NaiveWorkspaceCropper(roi_generator::Roi roi)
: mRoi(roi)
{
  //Do Nothing
}

void NaiveWorkspaceCropper::crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  ROS_INFO("Crop Ground");

  utils::filterCloud(cloud, "x", mRoi.x() - mRoi.length()/2.0, mRoi.x() + mRoi.length()/2.0);
  utils::filterCloud(cloud, "y", mRoi.y() - mRoi.width()/2.0, mRoi.y() + mRoi.width()/2.0);
  utils::filterCloud(cloud, "z", mRoi.z() - mRoi.height()/2.0, mRoi.z() + mRoi.height()/2.0);


  ROS_INFO("Ground Removed");

}


}
}