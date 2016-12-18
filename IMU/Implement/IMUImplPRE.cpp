#include "IMUImplPRE.h"

IMUImplPRE::IMUImplPRE()
{

}

int IMUImplPRE::propagation(const ImuMeasureDeque &imuMeasurements, const ImuParamenters &imuParams, Transformation &T_WS, SpeedAndBias &speedAndBiases, double &t_start, double &t_end, covariance_t *covariance, jacobian_t *jacobian)
{
    return 0;
}

