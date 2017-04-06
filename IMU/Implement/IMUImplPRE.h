#ifndef IMUIMPLPRE_H
#define IMUIMPLPRE_H

#include "IMUImpl.h"

class IMUImplPRE : public IMUImpl {
public:
    IMUImplPRE();
    ///< factor jacobian_t: dRdb_g, dvdb_a dvdb_g dpdb_a dpdb_g
    int propagation(const ImuMeasureDeque & imuMeasurements,
                    const ImuParameters & imuParams,
                    Transformation& T_WS,
                    SpeedAndBias & speedAndBiases,
                    okvis::Time &t_start,
                    okvis::Time &t_end,
                    covariance_t* covariance,
                    jacobian_t* jacobian);

    int error(const IMU::pViFrame &frame_i, const IMU::pViFrame &frame_j, Error_t &err, void *info);
    /**
     * @brief Jacobian
     * @param jacobian_t: drdphi drdb, dvdphi dvdv dvdb, dpdphi dpdv dpdp dpdb, for bias i->b_g, j->b_a
     */
    int Jacobian(const Error_t& err, const IMU::pViFrame& frame_i, jacobian_t& jacobian_i, const IMU::pViFrame& frame_j, jacobian_t& jacobian_j, void *info);
};

#endif // IMUIMPLPRE_H
