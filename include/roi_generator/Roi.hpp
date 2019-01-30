//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_ROI_HPP
#define PROJECT_ROI_HPP

#include <Eigen/Dense>

namespace roi_generator {

class Roi {
public:

  Roi (double x, double y, double z, double length, double width, double height) :
    mX(x),
    mY(y),
    mZ(z),
    mLength(length),
    mWidth(width),
    mHeight(height){}

  Roi() :
    mX(0),
    mY(0),
    mZ(0),
    mLength(0),
    mWidth(0),
    mHeight(0){}

  Roi(double length, double width, double height) :
    mX(0),
    mY(0),
    mZ(0),
    mLength(length),
    mWidth(width),
    mHeight(height){}

  double& x() {return mX;}
  double& y() {return mY;}
  double& z() {return mZ;}
  double& length() {return mLength;}
  double& width() {return mWidth;}
  double& height() {return mHeight;}

private:
  double mX,mY,mZ,mLength,mWidth,mHeight;
};

}

#endif //PROJECT_ROI_HPP
