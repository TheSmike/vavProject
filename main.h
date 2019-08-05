#ifndef _MAIN_H
#define _MAIN_H

#include <iostream>
#include <Eigen/SVD>
#include <Eigen/LU>




#include <map>

//#include <ceres/ceres.h>
//#include <ceres/rotation.h>

using namespace std;

const char *DS_FILE = "/home/studente/projects/file/ba_storage.txt";
// const char *DS_FILE = "/home/studente/projects/file/my_match.txt";

const double f = 718.856; //lunghezza focale
const double u0 = 607.193; //centro ottico x
const double v0 = 185.216; //centro ottico y
const double b = 0.537166; //distanza tra le 2 camere

//f = 718.856; u0 = 607.193; v0 = 185.216; b = 0.537166


double vo[6];


class KeyPointMatch{
  
public: 
  Eigen::Vector2d left;
  Eigen::Vector2d right;
  Eigen::Vector3d pt3d;
  
  //peso da dare al punto (probabilità di essere un outlier)
  double w;
  
  KeyPointMatch(){
  }
  
  KeyPointMatch(float xl, float yl, float xr, float yr, double w){
    left = Eigen::Vector2d(xl,yl);
    right = Eigen::Vector2d(xr, yr);
    pt3d = Eigen::Vector3d(0.0f,0.0f,0.0f);
    this->w = w;
  }
  
  void set3d(float x, float y, float z){
    this->pt3d[0] = x;
    this->pt3d[1] = y;
    this->pt3d[2] = z;
  }
  
};

std::ostream& operator<<(std::ostream &strm, const KeyPointMatch &a) {
  return strm << "left = (" << a.left.x() << ", " << a.left.y() <<  "); " << "right = (" << a.right.x() << ", " << a.right.y() 
	      <<  "); 3D = (" << a.pt3d.x() << ", " << a.pt3d.y() << ", " << a.pt3d.z() << "); w = " << a.w << ".";
  
}

  
#endif