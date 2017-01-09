//
// Created by lancelot on 1/4/17.
//

#ifndef SIMPLE_VIO_IMUG2OTYPE_H
#define SIMPLE_VIO_IMUG2OTYPE_H


#include <memory>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "ThirdParty/sophus/so3.hpp"
#include "Eigen/Dense"

class viFrame;
class imuFactor;
class ImuParameters;
class AbstractCamera;

struct camEdgeData {
    double u;
    double v;

    const camEdgeData& operator=(const camEdgeData& oth) {
        u = oth.u;
        v = oth.v;
    }
};


struct viVertexData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sophus::SO3d     orient;
    Eigen::Vector3d  pos;
    Eigen::Vector3d  speed;
    Eigen::Vector3d  bias_g;
    Eigen::Vector3d  bias_a;
    const viVertexData& operator=(const viVertexData& oth) {
        orient = oth.orient;
        pos    = oth.pos;
        speed  = oth.speed;
        bias_g = oth.bias_g;
        bias_a = oth.bias_a;
    }
};

struct imuEdgeData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sophus::SO3d     rotation;
    Eigen::Vector3d  translation;
    Eigen::Vector3d  dSpeed;

    const imuEdgeData& operator=(const imuEdgeData&oth) {
        rotation    = oth.rotation;
        translation = oth.translation;
        dSpeed      = oth.dSpeed;
    }

};


class viPREVertex : public g2o::BaseVertex<15, viVertexData> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    viPREVertex() {}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void setEstimateData(std::shared_ptr<viFrame>& viframe);
    virtual void oplusImpl(const double* update);
    virtual void setToOriginImpl();
};

class viPREEdge : public g2o::BaseBinaryEdge<9, imuEdgeData, viPREVertex, viPREVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    viPREEdge(){}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    bool setMeasurementData(const std::shared_ptr<imuFactor> &imufactor,
                            double dt, std::shared_ptr<ImuParameters>& imuParam,
                            const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> *covariance = nullptr);
    void linearizeOplus();
    virtual void computeError();

private:
    Eigen::Matrix<double, 15, 3>   JBias;
    double                         dt;
    std::shared_ptr<ImuParameters> imuParam;
};


class viCamEdge : public g2o::BaseBinaryEdge<2, camEdgeData, viPREVertex, g2o::VertexPointXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    viCamEdge() {};
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    bool setMeasurementData(const camEdgeData &pixel,
                            std::shared_ptr<AbstractCamera>& cam,
                            const Eigen::Matrix2d *covariance = nullptr);

    void linearizeOplus();
    virtual void computeError();

private:
    std::shared_ptr<AbstractCamera> cam;
};


#endif //SIMPLE_VIO_IMUG2OTYPE_H
