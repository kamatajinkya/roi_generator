//
// Created by ajinkya on 1/23/19.
//

#ifndef PROJECT_ROI_HPP
#define PROJECT_ROI_HPP

#include <Eigen/Dense>
#include <string>

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

  std::string toString()
  {
    std::string comma(", ");
    return std::to_string(mX) + comma
         + std::to_string(mY) + comma
         + std::to_string(mZ) + comma
         + std::to_string(mLength) + comma
         + std::to_string(mWidth) + comma
         + std::to_string(mHeight) + std::string(";");
  }

private:
  double mX,mY,mZ,mLength,mWidth,mHeight;
};

}

#endif //PROJECT_ROI_HPP
