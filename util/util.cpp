#include "util.h"

#include "DataStructure/cv/Camera/AbstractCamera.h"

const double EPS = 0.0000000001;
const double PI = 3.14159265;

Eigen::Matrix3d leftJacobian(const Eigen::Vector3d & PhiVec){
    return rightJacobian(-PhiVec);
}

Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &PhiVec) {
    const double Phi = PhiVec.norm();
    Eigen::Matrix3d retMat = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Phi_x =  crossMx(PhiVec);
    const Eigen::Matrix3d Phi_x2 = Phi_x*Phi_x;
    if(Phi < 1.0e-4) {
        retMat += -0.5*Phi_x + 1.0/6.0*Phi_x2;
    } else {
        const double Phi2 = Phi*Phi;
        const double Phi3 = Phi2*Phi;
        retMat += -(1.0-cos(Phi))/(Phi2)*Phi_x + (Phi-sin(Phi))/Phi3*Phi_x2;
    }
    return retMat;
}

double shiTomasiScore(const std::vector<Eigen::Matrix<double, 3, 1>>& img,
                      int width, int height, int u, int v) {
    typedef const std::vector<Eigen::Matrix<double, 3, 1>> VecImg;
    double dXX = 0.0;
    double dYY = 0.0;
    double dXY = 0.0;

    const int halfbox_size = 4;
    const int box_size = 2*halfbox_size;
    const int box_area = box_size*box_size;
    const int x_min = u-halfbox_size;
    const int x_max = u+halfbox_size;
    const int y_min = v-halfbox_size;
    const int y_max = v+halfbox_size;

    if(x_min < 1 || x_max >= width - 1 || y_min < 1 || y_max >= height - 1)
        return 0.0; // patch is too close to the boundary

    const int stride = width;
    for( int y=y_min; y<y_max; ++y )
    {
        VecImg::const_iterator ptr_left   = img.begin() + stride*y + x_min - 1;
        VecImg::const_iterator ptr_right  = img.begin()+ stride*y + x_min + 1;
        VecImg::const_iterator ptr_top    = img.begin() + stride*(y-1) + x_min;
        VecImg::const_iterator ptr_bottom = img.begin() + stride*(y+1) + x_min;
        for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
        {
            double dx = (*ptr_right)(0) - (*ptr_left)(0);
            double dy = (*ptr_bottom)(0) - (*ptr_top)(0);
            dXX += dx*dx;
            dYY += dy*dy;
            dXY += dx*dy;
        }
    }

    // Find and return smaller eigenvalue:
    dXX = dXX / (2.0 * box_area);
    dYY = dYY / (2.0 * box_area);
    dXY = dXY / (2.0 * box_area);
    return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));

}

float shiTomasiScore(const cv::Mat& img, int u, int v) {
  assert(img.type() == CV_8UC1);

  float dXX = 0.0;
  float dYY = 0.0;
  float dXY = 0.0;
  const int halfbox_size = 4;
  const int box_size = 2*halfbox_size;
  const int box_area = box_size*box_size;
  const int x_min = u-halfbox_size;
  const int x_max = u+halfbox_size;
  const int y_min = v-halfbox_size;
  const int y_max = v+halfbox_size;

  if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
    return 0.0; // patch is too close to the boundary

  const int stride = img.step.p[0];
  for( int y=y_min; y<y_max; ++y )
  {
    const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
    const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
    const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
    const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
    for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
    {
      float dx = *ptr_right - *ptr_left;
      float dy = *ptr_bottom - *ptr_top;
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }

  // Find and return smaller eigenvalue:
  dXX = dXX / (2.0 * box_area);
  dYY = dYY / (2.0 * box_area);
  dXY = dXY / (2.0 * box_area);
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}


cv::Mat Undistort(const cv::Mat& src, std::shared_ptr<AbstractCamera> cam) {
    cv::Mat dst;
    const Eigen::Matrix<double, 3, 3> &K_ = cam->K();
    cv::Mat K(3, 3, CV_64FC1);
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j)
            K.at<double>(i, j) = K_(i, j);
    }
    cv::Mat distCoeffs(4, 1, CV_64FC1);
    for(int i = 0; i < 4; ++i)
        distCoeffs.at<double>(i, 0) = cam->d(i);

    cv::undistort(src, dst, K, distCoeffs);

    return dst;
};