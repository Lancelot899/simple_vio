#include "util.h"

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
