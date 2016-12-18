#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Dense>

template<typename Scalar_T>
inline Eigen::Matrix<Scalar_T, 3, 3> crossMx(Scalar_T x, Scalar_T y, Scalar_T z);


#endif // UTIL_H
