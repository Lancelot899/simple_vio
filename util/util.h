#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


extern const double EPS;
extern const double PI;

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

template<typename Derived_T>
inline Eigen::Matrix<typename Eigen::internal::traits<Derived_T>::Scalar, 3, 3> crossMx(
        Eigen::MatrixBase<Derived_T> const & v) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_T>, 3);
    assert((v.cols()==3 && v.rows()==1)||(v.rows()==3 && v.cols()==1));
    return crossMx(v(0, 0), v(1, 0), v(2, 0));
}

Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & PhiVec);
float shiTomasiScore(const cv::Mat& img, int u, int v);

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


#endif // UTIL_H
