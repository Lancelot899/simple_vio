#include "util.h"

const double EPS = 0.0000000001;
const double PI = 3.14159265;


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
