//
// Created by ajinkya on 1/29/19.
//

#ifndef PROJECT_UTILS_HPP
#define PROJECT_UTILS_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace roi_generator {
namespace utils {

void filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 std::string filterName,
                 double minFilterLimit,
                 double maxFilterLimit);

}
}

#endif //PROJECT_UTILS_HPP
