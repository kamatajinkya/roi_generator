//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_PCDPOINTCLOUDRECEIVER_HPP
#define PROJECT_PCDPOINTCLOUDRECEIVER_HPP

#include "AbstractPointCloudReciever.hpp"

namespace roi_generator {
namespace pointCouldReceiver {

class PCDPointCloudReceiver : public AbstractPointCloudReceiver{
public:
  PCDPointCloudReceiver(std::string fileName);
  pcl::PointCloud<pcl::PointXYZRGB> receive() override;

private:
  std::string mFileName;
};

}
}



#endif //PROJECT_PCDPOINTCLOUDRECEIVER_HPP
