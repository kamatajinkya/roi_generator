//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_NAIVEFLOORREMOVER_HPP
#define PROJECT_NAIVEFLOORREMOVER_HPP

#include "AbstractFloorRemover.hpp"

namespace roi_generator {
namespace floorRemover {

class NaiveFloorRemover : public AbstractFloorRemover {
public:
  NaiveFloorRemover(double floorHeight);

  void remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;

private:
  double mFloorHeight;

};

}
} //
#endif //PROJECT_NAIVEFLOORREMOVER_HPP
