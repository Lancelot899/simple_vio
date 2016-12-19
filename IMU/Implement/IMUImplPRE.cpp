#include "IMUImplPRE.h"
#include "DataStructure/IMUMeasure.h"
#include "util/util.h"

IMUImplPRE::IMUImplPRE()
{

}

int IMUImplPRE::propagation(const ImuMeasureDeque &imuMeasurements,
                            const ImuParamenters &imuParams,
                            Transformation &T_WS, SpeedAndBias &speedAndBiases,
                            double &t_start, double &t_end,
                            covariance_t *covariance, jacobian_t *jacobian) {
    double time = t_start;
    double end = t_end;

    assert(imuMeasurements.front().timeStamp<=time);
    if (!(imuMeasurements.back().timeStamp >= end))
        return -1;

    const Eigen::Vector3d gbias = speedAndBiases.segment<3>(3);
    const Eigen::Vector3d abias = speedAndBiases.segment<3>(6);
    Sophus::SO3d D_rotation(Eigen::Quaternion<double>(1, 0, 0, 0));
    Eigen::Vector3d D_vec(0, 0, 0);
    Eigen::Vector3d D_pos(0, 0, 0);

    double Delta_t = 0;
    bool hasStarted = false;
    int i = 0;

    for(auto it = imuMeasurements.begin(); it + 1 != imuMeasurements.end(); ++it) {
        Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
        Eigen::Vector3d acc_S_0 = it->measurement.acceleration;
        Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
        Eigen::Vector3d acc_S_1 = (it + 1)->measurement.acceleration;

        double nexttime;
        if((it + 1) == imuMeasurements.end())
            nexttime = t_end;
        else
            nexttime = (it + 1)->timeStamp;

        double dt = nexttime - time;

        if (end < nexttime) {
            double interval = nexttime - it->timeStamp;
            nexttime = t_end;
            dt = nexttime - time;
            const double r = dt / interval;
            omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
            acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
        }

        if (dt <= 0.0) continue;
        Delta_t += dt;

        if (!hasStarted) {
            hasStarted = true;
            const double r = dt / (nexttime - it->timeStamp);
            omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
            acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
        }

        double sigma_g_c = imuParams.sigma_g_c;
        double sigma_a_c = imuParams.sigma_a_c;

        if (fabs(omega_S_0[0]) > imuParams.g_max
                || fabs(omega_S_0[1]) > imuParams.g_max
                || fabs(omega_S_0[2]) > imuParams.g_max
                || fabs(omega_S_1[0]) > imuParams.g_max
                || fabs(omega_S_1[1]) > imuParams.g_max
                || fabs(omega_S_1[2]) > imuParams.g_max) {
            sigma_g_c *= 100;
        }

        if (fabs(acc_S_0[0]) > imuParams.a_max || fabs(acc_S_0[1]) > imuParams.a_max
                || fabs(acc_S_0[2]) > imuParams.a_max
                || fabs(acc_S_1[0]) > imuParams.a_max
                || fabs(acc_S_1[1]) > imuParams.a_max
                || fabs(acc_S_1[2]) > imuParams.a_max) {
            sigma_a_c *= 100;
        }

        Sophus::SO3d dR = Sophus::SO3d::exp((0.5*(omega_S_0 + omega_S_1) - gbias) * dt);
        D_rotation *= dq;
        const Eigen::Vector3d acc_S_true = 0.5*(acc_S_0+acc_S_1) - abias;
        Eigen::Vector3d dv = D_rotation * (acc_S_true * dt);
        D_vec += dv;
        Eigen::Vector3d dp = 1.5 * D_rotation * (acc_S_true) * dt * dt;
        D_pos += dp;

        if (covariance) {
            assert(covariance->cols() = 9 && covariance->rows() = 9);
            Eigen::Matrix<double, 9, 6> B;
            B.setZero();
            B.block<3, 3>(0, 0) = dt * rightJacobian(dR.log());
            B.block<3, 3>(3, 3) = dt * D_rotation;
            B.block<3, 3>(6, 3) = 0.5 * dt * dt * D_rotation;

            Eigen::Matrix<double, 9, 9> A;
            A.setZero();

            if(i != 0) {
                A.block<3, 3>(0, 0) = dR.inverse().matrix();
                A.block<3, 3>(3, 0) = -dt * D_rotation.matrix() * Sophus::SO3d::hat(acc_S_true);
                A.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
                A.block<3, 3>(6, 0) = -0.5 * dt * dt * D_rotation.matrix() * Sophus::SO3d::hat(acc_S_true);
                A.block<3, 3>(6, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
                A.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
            }
            else
                covariance->setZero();

            Eigen::Matrix<double, 6, 6> Cov_eta;
            Cov_eta.setOnes();
            Cov_eta.block<3, 3>(0, 0) = dt * sigma_g_c * Eigen::Matrix<double, 3, 3>::Identity();
            Cov_eta.block<3, 3>(3, 3) = dt * sigma_a_c * Eigen::Matrix<double, 3, 3>::Identity();
            Eigen::Matrix<double, 9, 9> &Cov = *covariance;
            Cov = A * Cov * A.transpose() + B * Cov_eta * B.transpose();
        }

        time = nexttime;

        if(jacobian) {

        }

        ++i;

        if (nexttime == t_end)
            break;
    }

    return 0;
}

