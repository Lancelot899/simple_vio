//
// Created by lancelot on 1/6/17.
//

#include "VIOPinholeCamera.h"

using namespace Eigen;

Vector3d VIOPinholeCamera::cam2world(const double &x, const double &y) const {
    return PinholeCamera::cam2world(x,y);
}

Vector3d VIOPinholeCamera::cam2world(const Vector2d &px) const {
    return PinholeCamera::cam2world(px);
//    tmp = T_BS * tmp;
//    tmp /= tmp[2];
//    return tmp;
}

Vector2d VIOPinholeCamera::world2cam(const Vector3d &xyz_c) const {
    return PinholeCamera::world2cam(xyz_c);
}

Vector2d VIOPinholeCamera::world2cam(const Vector2d &uv) const {
    return PinholeCamera::world2cam(Vector3d(uv[0],uv[1],1));
}
