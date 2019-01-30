//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_NAIVEWORKSPACECROPPER_HPP
#define PROJECT_NAIVEWORKSPACECROPPER_HPP

#include "AbstractWorkspaceRemover.hpp"
#include "roi_generator/Roi.hpp"

namespace roi_generator {
namespace workspaceCropper {

class NaiveWorkspaceCropper : public AbstractWorkspaceCropper {
public:
  NaiveWorkspaceCropper(roi_generator::Roi roi);
  void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;
private:
  roi_generator::Roi mRoi;
};

}
}

#endif //PROJECT_NAIVEWORKSPACECROPPER_HPP
