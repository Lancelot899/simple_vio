#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Dense>

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


#endif // UTIL_H
