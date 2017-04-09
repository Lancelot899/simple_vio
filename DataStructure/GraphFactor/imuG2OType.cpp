//
// Created by lancelot on 1/4/17.
//

#include "DataStructure/GraphFactor/imuG2OType.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "IMU/IMU.h"
#include "util/util.h"
#include "DataStructure/cv/Camera/VIOPinholeCamera.h"

bool viPREVertex::read(std::istream &is) {return true;}
bool viPREVertex::write(std::ostream &os) const {return true;}

void viPREVertex::setEstimateData(std::shared_ptr<viFrame> &viframe) {
    if(viframe.get() == nullptr)
        return;
    _estimate.orient = viframe->getPose().so3();
    _estimate.pos    = viframe->getPose().translation();
    _estimate.speed  = viframe->getSpeedAndBias().segment<3>(0);
    _estimate.bias_g = viframe->getSpeedAndBias().segment<3>(3);
    _estimate.bias_a = viframe->getSpeedAndBias().segment<3>(6);
}

void viPREVertex::oplusImpl(const double *update) {
    const Eigen::Matrix<double, 15, 1> v(update);
    _estimate.pos    += _estimate.orient * v.block<3, 1>(3, 0);
    _estimate.orient *= Sophus::SO3d::exp(v.block<3 ,1>(0, 0));
    _estimate.speed  += v.block<3, 1>(6, 0);
    _estimate.bias_g += v.block<3, 1>(9, 0);
    _estimate.bias_a += v.block<3, 1>(12, 0);
}

void viPREVertex::setToOriginImpl() {
    _estimate.orient = Sophus::SO3d();
    _estimate.pos    = Eigen::Vector3d::Zero();
    _estimate.speed  = Eigen::Vector3d::Zero();
    _estimate.bias_g = Eigen::Vector3d::Zero();
    _estimate.bias_a = Eigen::Vector3d::Zero();
}

bool viPREEdge::read(std::istream& is) {
    return true;
}

bool viPREEdge::write(std::ostream& os) const {
    return true;
}

bool viPREEdge::setMeasurementData(const std::shared_ptr<imuFactor> &imufactor,
                                   double dt, std::shared_ptr<ImuParameters>& imuParam,
                                   const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> *covariance) {
    if(imufactor.get() == nullptr)
        return false;
    _measurement.rotation    = imufactor->getPoseFac().so3();
    _measurement.translation = imufactor->getPoseFac().translation();
    _measurement.dSpeed      = imufactor->getSpeedFac();

    if(covariance == nullptr)
        _information = Eigen::Matrix<double, 9, 9>::Identity();
    else {
        _information = covariance->inverse();
    }

    JBias  = imufactor->getJBias();
    this->dt = dt;
    this->imuParam = imuParam;

    return true;
}

void viPREEdge::computeError() {
    const viVertexData &v1 = (static_cast<viPREVertex*>(_vertices[0]))->estimate();
    const viVertexData &v2 = (static_cast<viPREVertex*>(_vertices[1]))->estimate();
    Eigen::Vector3d dbias_g = v2.bias_g - v1.bias_g;
    Eigen::Vector3d dbias_a = v2.bias_a - v1.bias_a;
    _error.segment<3>(0) = Sophus::SO3d::log(_measurement.rotation *
                                                     Sophus::SO3d::exp(JBias.block<3, 3>(0, 0) * dbias_g) *
                                                    v1.orient.inverse() * v2.orient);
    _error.segment<3>(3) = v1.orient.inverse() * (v2.speed - v1.speed - imuParam->g * dt)
                            - (_measurement.dSpeed + JBias.block<3, 3>(6, 0) * dbias_g
                                                         + JBias.block<3, 3>(3, 0) * dbias_a);
    _error.segment<3>(6) = v1.orient.inverse() * (v2.pos - v1.pos - v1.speed * dt - 0.5 * imuParam->g * dt * dt)
                            - (_measurement.translation + JBias.block<3, 3>(12, 0) * dbias_g
                                                                     +  JBias.block<3, 3>(9, 0) * dbias_a);
}


void viPREEdge::linearizeOplus() {
    const viVertexData &v1 = (static_cast<viPREVertex*>(_vertices[0]))->estimate();
    const viVertexData &v2 = (static_cast<viPREVertex*>(_vertices[1]))->estimate();
    Eigen::Vector3d dbias_g = v2.bias_g - v1.bias_g;
    _jacobianOplusXi.block<3, 3>(0, 0) = -rightJacobian(_error.segment<3>(0)) * (v2.orient.inverse() * v1.orient).matrix();
    _jacobianOplusXi.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(0, 9) = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(0, 12) = Eigen::Matrix3d::Zero();

    _jacobianOplusXj.block<3, 3>(0, 0) = rightJacobian(_error.segment<3>(0));
    _jacobianOplusXj.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    _jacobianOplusXj.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
    _jacobianOplusXj.block<3, 3>(0, 9) = -rightJacobian(_error.segment<3>(0)) * Sophus::SO3d::exp(_error.segment<3>(0)).inverse().matrix()
                                            * rightJacobian(JBias.block<3, 3>(0, 0) * dbias_g) * JBias.block<3, 3>(0, 0);
    _jacobianOplusXj.block<3, 3>(0, 12) = Eigen::Matrix3d::Zero();

    _jacobianOplusXi.block<3, 3>(3, 0) = Sophus::SO3d::hat(v1.orient.inverse() * (v2.speed - v1.speed - imuParam->g  * dt));
    _jacobianOplusXj.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
    _jacobianOplusXj.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(3, 6) = -v1.orient.inverse().matrix();
    _jacobianOplusXj.block<3, 3>(3, 6) = v1.orient.inverse().matrix();
    _jacobianOplusXi.block<3, 3>(3, 9)  = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(3, 12) = Eigen::Matrix3d::Zero();
    _jacobianOplusXj.block<3, 3>(3, 9)  = -JBias.block<3, 3>(6, 0);
    _jacobianOplusXj.block<3, 3>(3, 12) = -JBias.block<3, 3>(3, 0);

    _jacobianOplusXi.block<3, 3>(6, 0) = Sophus::SO3d::hat(v1.orient.inverse() *
                                                           (v2.pos - v1.pos - v1.speed * dt - 0.5 * imuParam->g  * dt * dt));
    _jacobianOplusXj.block<3, 3>(6, 0)  = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(6, 3)  = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.block<3, 3>(6, 3)  = (v1.orient.inverse() * v2.orient).matrix();
    _jacobianOplusXi.block<3, 3>(6, 6)  = -v1.orient.inverse().matrix() * dt;
    _jacobianOplusXj.block<3, 3>(6, 6)  = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(6, 9)  = Eigen::Matrix3d::Zero();
    _jacobianOplusXi.block<3, 3>(6, 12) = Eigen::Matrix3d::Zero();
    _jacobianOplusXj.block<3, 3>(6, 9)  = -JBias.block<3, 3>(12, 0);
    _jacobianOplusXj.block<3, 3>(6, 12) = -JBias.block<3, 3>(9, 0);
}

bool viCamEdge::read(std::istream &is) {
    return true;
}

bool viCamEdge::write(std::ostream &os) const {
    return true;
}

bool viCamEdge::setMeasurementData(const camEdgeData &pixel, std::shared_ptr<AbstractCamera> &cam,
                                   const Eigen::Matrix2d *covariance) {
    if(cam.get() == nullptr)
        return false;

    _measurement = pixel;
    this->cam = cam;
    if(covariance != nullptr)
        _information = covariance->inverse();
    else
        _information = Eigen::Matrix2d::Identity();

    return true;

}
void viCamEdge::computeError()
{
    const viVertexData &v1 = (static_cast<viPREVertex*>(_vertices[0]))->estimate();
    const g2o::Vector3D &v2 = (static_cast<g2o::VertexPointXYZ*>(_vertices[1]))->estimate();
    g2o::Vector3D camPoint = v1.orient * v2 + v1.pos;

    _error[0] = _measurement.u - cam->fx()*camPoint[0]/camPoint[2] - cam->cx();
    _error[1] = _measurement.v - cam->fy()*camPoint[1]/camPoint[2] - cam->cy();
}
void viCamEdge::linearizeOplus()
{
    const viVertexData &v1 = (static_cast<viPREVertex*>(_vertices[0]))->estimate();
    const g2o::Vector3D &v2 = (static_cast<g2o::VertexPointXYZ*>(_vertices[1]))->estimate();
    g2o::Vector3D camPoint = v1.orient * v2 + v1.pos ;
    g2o::Vector3D camPoint0 = camPoint/camPoint[2];

    double depth_inv = 1.0/camPoint[2];
    _jacobianOplusXi.block<2,3>(0,0) << cam->fx()*camPoint[0]*camPoint[1]*depth_inv*depth_inv,   -cam->fx()-cam->fx()*camPoint[0]*camPoint[0]*depth_inv*depth_inv,  cam->fx()*camPoint[1]*depth_inv,
                                        cam->fy()+cam->fy()*camPoint[1]*camPoint[1]*depth_inv*depth_inv,  -cam->fy()*camPoint[0]*camPoint[1]*depth_inv*depth_inv,  -cam->fy()*camPoint[0]*depth_inv;

    _jacobianOplusXi.block<2,3>(0,3) << -cam->fx()*depth_inv, 0.0,                   cam->fx()*camPoint[0]*depth_inv*depth_inv,
                                        0.0,   -cam->fy()*depth_inv, cam->fy()*camPoint[1]*depth_inv*depth_inv;

    _jacobianOplusXi.block<2,9>(0,6) = Eigen::Matrix<double,2,9>::Zero();



    Eigen::Matrix<double,2,3> KInvd;
    KInvd <<-cam->fx()/camPoint[2], 0,                     cam->fx()*camPoint[0]/(camPoint[2]*camPoint[2]),
            0,                     -cam->fy()/camPoint[2], cam->fy()*camPoint[1]/(camPoint[2]*camPoint[2]);

    _jacobianOplusXi.block<2,3>(0,0) = KInvd*v1.orient.matrix();

}
