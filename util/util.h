#ifndef UTIL_H
#define UTIL_H

#include <memory>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

extern const double EPS;
extern const double PI;
#define DEBUG_VIO printf("file[%s], line[%d]\n",__FILE__, __LINE__)

class AbstractCamera;

typedef Eigen::Matrix<double, 2, 3>  Matrix23d;

template<typename Scalar_T>
inline Eigen::Matrix<Scalar_T, 3, 3> crossMx(Scalar_T x, Scalar_T y, Scalar_T z)
{
    Eigen::Matrix<Scalar_T, 3, 3> C;
    C(0, 0) = 0.0;
    C(0, 1) = -z;
    C(0, 2) = y;
    C(1, 0) = z;
    C(1, 1) = 0.0;
    C(1, 2) = -x;
    C(2, 0) = -y;
    C(2, 1) = x;
    C(2, 2) = 0.0;
    return C;
}


inline Eigen::Vector3d
interpolate(std::vector<Eigen::Vector3d>::const_iterator& img, double u, double v, int cols)
{
    int x = int(floor(u));
    int y = int(floor(v));
    double subpix_x = u-x;
    double subpix_y = v-y;

    double w00 = (1.0f-subpix_x)*(1.0f-subpix_y);
    double w01 = (1.0f-subpix_x)*subpix_y;
    double w10 = subpix_x*(1.0f-subpix_y);
    double w11 = 1.0f - w00 - w01 - w10;

    std::vector<Eigen::Vector3d>::const_iterator ptr;
    ptr = img + y * cols + x;
    return w00 * (*ptr) + w01 * (*(ptr + cols)) + w10 * (*(ptr + 1)) + w11 * (*(ptr + cols + 1));
}

template<typename Derived_T>
inline Eigen::Matrix<typename Eigen::internal::traits<Derived_T>::Scalar, 3, 3> crossMx(
        Eigen::MatrixBase<Derived_T> const & v) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_T>, 3);
    assert((v.cols()==3 && v.rows()==1)||(v.rows()==3 && v.cols()==1));
    return crossMx(v(0, 0), v(1, 0), v(2, 0));
}

Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & PhiVec);
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d & PhiVec);
float shiTomasiScore(const cv::Mat& img, int u, int v);
double shiTomasiScore(const std::vector<Eigen::Matrix<double, 3, 1>>& img, int width, int height, int u, int v);

inline double norm_max(const Eigen::VectorXd & v)
{
    double max = -1;
    for (int i=0; i<v.size(); i++)
    {
        double abs = fabs(v[i]);
        if(abs>max){
            max = abs;
        }
    }
    return max;
}

inline Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
    return v.head<2>()/v[2];
}

inline Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
    return Eigen::Vector3d(v[0], v[1], 1.0);
}

inline Eigen::Vector3d project3d(const Eigen::Vector4d& v) {
    return v.head<3>()/v[3];
}

inline Eigen::Vector4d unproject3d(const Eigen::Vector3d& v) {
    return Eigen::Vector4d(v[0], v[1], v[2], 1.0);
}

class TimeUse
{
public:
    TimeUse(const char *function_= nullptr, int line_ = 0)
        :function_T(function_),line_T(line_)
    {
        gettimeofday(&start_T,0);
    }
    ~TimeUse(){
        gettimeofday(&end_T,0);
        double timeUse = double(end_T.tv_sec - start_T.tv_sec)*1000.0 + double(end_T.tv_usec - start_T.tv_usec)/1000.0;
        if(line_T && function_T!=nullptr)
            printf("\t--Time use in FUNCTION[%s], LINE[%d], is %f ms\n\n", function_T, line_T, timeUse);
        else
            printf("\t--Time use is %f ms\n\n", timeUse);
    }

protected:
    timeval           start_T,end_T;
    const char       *function_T;
    int               line_T;
};

cv::Mat Undistort(const cv::Mat& src, std::shared_ptr<AbstractCamera> cam);

extern double scale;
double init_depth();

#endif // UTIL_H
