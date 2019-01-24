//
// Created by ajinkya on 1/24/19.
//

#ifndef PROJECT_NAIVEWORKSPACECROPPER_HPP
#define PROJECT_NAIVEWORKSPACECROPPER_HPP

#include "AbstractWorkspaceRemover.hpp"

namespace roi_generator {
namespace workspaceCropper {

class NaiveWorkspaceCropper : public AbstractWorkspaceCropper {
public:
  void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) override;
};

}
}

#endif //PROJECT_NAIVEWORKSPACECROPPER_HPP
