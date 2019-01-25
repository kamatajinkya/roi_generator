//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_RVIZROIVISUALIZER_HPP
#define PROJECT_RVIZROIVISUALIZER_HPP

#include <map>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include "roi_generator/Roi.hpp"

namespace roi_generator {
namespace debugUtils {

class RVizRoiVisualizer {
public:

  RVizRoiVisualizer(ros::NodeHandle nh);
  int addRoi(Roi& roi, float red, float green, float blue);
  void removeRoi(int roiId);
  void visualize();

private:

  ros::NodeHandle mNh;
  ros::Publisher mPublisher;
  std::map<int, visualization_msgs::Marker> mMarkerArray;

};

}
}

#endif //PROJECT_RVIZROIVISUALIZER_HPP
