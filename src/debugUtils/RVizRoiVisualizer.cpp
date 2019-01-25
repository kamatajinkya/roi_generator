//
// Created by ajinkya on 1/24/19.
//

#include <utility>
#include <map>
#include "roi_generator/debugUtils/RVizRoiVisualizer.hpp"

namespace roi_generator {
namespace debugUtils {

RVizRoiVisualizer::RVizRoiVisualizer(ros::NodeHandle nh)
: mNh(nh)
{
  mPublisher = mNh.advertise<visualization_msgs::Marker>("roi_marker", 1);

}

int RVizRoiVisualizer::addRoi(Roi &roi, float red, float green, float blue) {

  unsigned int i = 1; // or whatever your smallest admissable key value is
  for (auto it = mMarkerArray.cbegin(), end = mMarkerArray.cend();
       it != end && i == it->first; ++it, ++i);

  visualization_msgs::Marker roi_marker;
  roi_marker.header.frame_id = "/map";
  roi_marker.header.stamp = ros::Time::now();
  roi_marker.ns = "roi";
  roi_marker.id = i;
  roi_marker.type = visualization_msgs::Marker::CUBE;
  roi_marker.action = visualization_msgs::Marker::ADD;
  roi_marker.pose.position.x = roi.x();
  roi_marker.pose.position.y = roi.y();
  roi_marker.pose.position.z = roi.z();
  roi_marker.pose.orientation.x = 0.0;
  roi_marker.pose.orientation.y = 0.0;
  roi_marker.pose.orientation.z = 0.0;
  roi_marker.pose.orientation.w = 1.0;
  roi_marker.scale.x = roi.length();
  roi_marker.scale.y = roi.width();
  roi_marker.scale.z = roi.height();
  roi_marker.color.r = red;
  roi_marker.color.g = green;
  roi_marker.color.b = blue;
  roi_marker.color.a = 1.0;
  roi_marker.lifetime = ros::Duration();
  mMarkerArray.insert(std::pair<int, visualization_msgs::Marker>(i, roi_marker));

  return i;
}

void RVizRoiVisualizer::removeRoi(int roiId) {
  if(mMarkerArray.find(roiId) != mMarkerArray.end())
    mMarkerArray.erase(roiId);
}

void RVizRoiVisualizer::visualize() {
  for(auto& marker:mMarkerArray)
    mPublisher.publish(marker.second);
}

}
}