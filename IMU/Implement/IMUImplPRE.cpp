#include "IMUImplPRE.h"
#include "DataStructure/IMUMeasure.h"
#include "util/util.h"
#include "DataStructure/viFrame.h"

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


    std::vector<double> VecDt;
    std::vector<Sophus::SO3d, Eigen::aligned_allocator<IMUMeasure>>    VecRotation;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<IMUMeasure>> VecRightJac;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<IMUMeasure>> VecAcc;

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

        VecDt.push_back(dt);

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
        Eigen::Matrix<double, 3, 3> rJac = rightJacobian(dR.log());
        VecRightJac.push_back(rJac);
        D_rotation *= dR;
        VecRotation.push_back(D_rotation);
        const Eigen::Matrix<double, 3, 3> D_rMat = D_rotation.matrix();

        const Eigen::Vector3d acc_S_true = 0.5*(acc_S_0+acc_S_1) - abias;
        VecAcc.push_back(acc_S_true);
        Eigen::Vector3d dv = D_rMat * (acc_S_true * dt);
        D_vec += dv;
        Eigen::Vector3d dp = 1.5 * D_rMat * (acc_S_true) * dt * dt;
        D_pos += dp;

        if (covariance) {
            assert(covariance->cols() == 9 && covariance->rows() == 9);
            Eigen::Matrix<double, 9, 6> B;
            B.setZero();
            B.block<3, 3>(0, 0) = dt * rJac;
            B.block<3, 3>(3, 3) = dt * D_rMat;
            B.block<3, 3>(6, 3) = 0.5 * dt * dt * D_rMat;

            Eigen::Matrix<double, 9, 9> A;
            A.setZero();

            if(i != 0) {
                A.block<3, 3>(0, 0) = dR.inverse().matrix();
                A.block<3, 3>(3, 0) = -dt * D_rMat * Sophus::SO3d::hat(acc_S_true);
                A.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
                A.block<3, 3>(6, 0) = -0.5 * dt * dt * D_rMat * Sophus::SO3d::hat(acc_S_true);
                A.block<3, 3>(6, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
                A.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
            }
            else
                covariance->setZero();

            Eigen::Matrix<double, 6, 6> Cov_eta;
            Cov_eta.setZero();
            Cov_eta.block<3, 3>(0, 0) = dt * sigma_g_c * Eigen::Matrix<double, 3, 3>::Identity();
            Cov_eta.block<3, 3>(3, 3) = dt * sigma_a_c * Eigen::Matrix<double, 3, 3>::Identity();
            *covariance = A * *covariance * A.transpose() + B * Cov_eta * B.transpose();
        }

        time = nexttime;

        ++i;

        if (nexttime == t_end)
            break;
    }


    if(jacobian) {
        assert(jacobian->rows() == 15 && jacobian->cols() == 3);
        jacobian->setZero();
        Sophus::SO3d R_ij = VecRotation[VecRotation.size()];
        Eigen::Matrix<double, 3, 3> matR_ij = R_ij.matrix();
        //VecRotation.pop_back();
        for(auto it = VecRotation.begin(); it != VecRotation.end(); ++it)
            *it = it->inverse() * R_ij;

        for(unsigned i = 0; i < VecRightJac.size(); ++i) {
            jacobian->block<3, 3>(0, 0) -= VecRotation[i].inverse().matrix() * VecRightJac[i] * VecDt[i];
            jacobian->block<3, 3>(3, 0) -= matR_ij * VecDt[i];
            jacobian->block<3, 3>(9, 0) -= 1.5 * matR_ij * VecDt[i];
        }

        const Eigen::Matrix<double, 3, 3> &dR_dbg = jacobian->block<3, 3>(0, 0);
        for(unsigned i = 0; i < VecAcc.size(); ++i) {
            jacobian->block<3, 3>(6, 0)  -= matR_ij * Sophus::SO3d::hat(VecAcc[i]) * dR_dbg * VecDt[i];
            jacobian->block<3, 3>(12, 0) -= 1.5 * matR_ij * Sophus::SO3d::hat(VecAcc[i]) * dR_dbg * VecDt[i] * VecDt[i];
        }

    }

    T_WS = Sophus::SE3d(D_rotation, D_pos);
    speedAndBiases.head<3>() = D_vec;

    return i;
}

int IMUImplPRE::error(const viFrame &frame_i, const viFrame &frame_j, Error_t &err, void *info)
{
    if(info == NULL)
        return -1;

    return 0;
}

int IMUImplPRE::Jacobian(const error_t &err, const viFrame &frame_i, jacobian_t &jacobian_i, const viFrame &frame_j, jacobian_t &jacobian_j)
{

    return 0;
}

