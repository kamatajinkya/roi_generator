//
// Created by ajinkya on 1/29/19.
//

#include "roi_generator/utils/utils.hpp"
#include <pcl/filters/passthrough.h>

namespace roi_generator {
namespace utils {

void filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 std::string filterName,
                 double minFilterLimit,
                 double maxFilterLimit)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ::pcl::PassThrough<pcl::PointXYZRGB> pass;

  pass.setInputCloud(cloud);
  pass.setFilterFieldName(filterName);
  pass.setFilterLimits(minFilterLimit, maxFilterLimit);
  pass.filter(*filteredCloud);

  *cloud = *filteredCloud;
}

}
}
