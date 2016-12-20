#include <assert.h>

#include "IMUImplOKVIS.h"
#include "util/util.h"

int IMUImplOKVIS::propagation(const ImuMeasureDeque &imuMeasurements,
                              const ImuParamenters &imuParams,
                              Transformation &T_WS,
                              SpeedAndBias &speedAndBiases,
                              double &t_start,
                              double &t_end,
                              covariance_t *covariance,
                              jacobian_t *jacobian) {
    double time = t_start;
    double end = t_end;

    assert(imuMeasurements.front().timeStamp<=time);
    if (!(imuMeasurements.back().timeStamp >= end))
        return -1;

    Eigen::Vector3d r_0 = T_WS.translation();
    Eigen::Quaterniond q_WS_0 = T_WS.so3().unit_quaternion();
    Eigen::Matrix3d C_WS_0 = T_WS.rotationMatrix();

    Eigen::Quaterniond Delta_q(1,0,0,0);
    Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
    Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

    Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();

    Eigen::Matrix<double,15,15> P_delta = Eigen::Matrix<double,15,15>::Zero();

    double Delta_t = 0;
    bool hasStarted = false;
    int i = 0;

    for(auto it = imuMeasurements.begin(); it != imuMeasurements.end(); ++it) {
        Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
        Eigen::Vector3d acc_S_0 = it->measurement.acceleration;
        Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
        Eigen::Vector3d acc_S_1 = (it + 1)->measurement.acceleration;

        double nexttime;
        if ((it + 1) == imuMeasurements.end()) {
            nexttime = t_end;
        } else
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

        if (dt <= 0.0) {
            continue;
        }
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

        Eigen::Quaterniond dq;
        const Eigen::Vector3d omega_S_true = (0.5*(omega_S_0+omega_S_1) - speedAndBiases.segment<3>(3));
        const double theta_half = omega_S_true.norm() * 0.5 * dt;
        const double sinc_theta_half = sin(theta_half);
        const double cos_theta_half = cos(theta_half);
        dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
        dq.w() = cos_theta_half;
        Eigen::Quaterniond Delta_q_1 = Delta_q * dq;

        const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
        const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
        const Eigen::Vector3d acc_S_true = (0.5*(acc_S_0+acc_S_1) - speedAndBiases.segment<3>(6));
        const Eigen::Matrix3d C_integral_1 = C_integral + 0.5*(C + C_1)*dt;
        const Eigen::Vector3d acc_integral_1 = acc_integral + 0.5*(C + C_1)*acc_S_true*dt;

        C_doubleintegral += C_integral*dt + 0.25*(C + C_1)*dt*dt;
        acc_doubleintegral += acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt;


        dalpha_db_g += dt*C_1;
        const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix()*cross + rightJacobian(omega_S_true*dt)*dt;
        const Eigen::Matrix3d acc_S_x = crossMx(acc_S_true);
        Eigen::Matrix3d dv_db_g_1 = dv_db_g + 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
        dp_db_g += dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);

        if (covariance) {
            assert(covariance->cols() == 15 && covariance->rows() == 15);
            Eigen::Matrix<double,15,15> F_delta = Eigen::Matrix<double,15,15>::Identity();

            F_delta.block<3,3>(0,3) = -crossMx(acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt);
            F_delta.block<3,3>(0,6) = Eigen::Matrix3d::Identity()*dt;
            F_delta.block<3,3>(0,9) = dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
            F_delta.block<3,3>(0,12) = -C_integral*dt + 0.25*(C + C_1)*dt*dt;
            F_delta.block<3,3>(3,9) = -dt*C_1;
            F_delta.block<3,3>(6,3) = -crossMx(0.5*(C + C_1)*acc_S_true*dt);
            F_delta.block<3,3>(6,9) = 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
            F_delta.block<3,3>(6,12) = -0.5*(C + C_1)*dt;
            P_delta = F_delta*P_delta*F_delta.transpose();
            const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
            P_delta(3,3) += sigma2_dalpha;
            P_delta(4,4) += sigma2_dalpha;
            P_delta(5,5) += sigma2_dalpha;
            const double sigma2_v = dt * sigma_a_c * imuParams.sigma_a_c;
            P_delta(6,6) += sigma2_v;
            P_delta(7,7) += sigma2_v;
            P_delta(8,8) += sigma2_v;
            const double sigma2_p = 0.5*dt*dt*sigma2_v;
            P_delta(0,0) += sigma2_p;
            P_delta(1,1) += sigma2_p;
            P_delta(2,2) += sigma2_p;
            const double sigma2_b_g = dt * imuParams.sigma_gw_c * imuParams.sigma_gw_c;
            P_delta(9,9)   += sigma2_b_g;
            P_delta(10,10) += sigma2_b_g;
            P_delta(11,11) += sigma2_b_g;
            const double sigma2_b_a = dt * imuParams.sigma_aw_c * imuParams.sigma_aw_c;
            P_delta(12,12) += sigma2_b_a;
            P_delta(13,13) += sigma2_b_a;
            P_delta(14,14) += sigma2_b_a;
        }

        Delta_q = Delta_q_1;
        C_integral = C_integral_1;
        acc_integral = acc_integral_1;
        cross = cross_1;
        dv_db_g = dv_db_g_1;
        time = nexttime;

        ++i;

        if (nexttime == t_end)
            break;
    }

    const Eigen::Vector3d g_W = imuParams.g * Eigen::Vector3d(0, 0, 6371009).normalized();
    T_WS = Sophus::SE3d(q_WS_0*Delta_q, r_0+speedAndBiases.head<3>()*Delta_t
                        + C_WS_0*(acc_doubleintegral)
                        - 0.5*g_W*Delta_t*Delta_t);
    speedAndBiases.head<3>() += C_WS_0*(acc_integral)-g_W*Delta_t;

    if(jacobian) {
        assert(jacobian->cols() == 15 && jacobian->rows() == 15);
        jacobian->setIdentity();
        jacobian->block<3,3>(0,3) = -crossMx(C_WS_0*acc_doubleintegral);
        jacobian->block<3,3>(0,6) = Eigen::Matrix3d::Identity()*Delta_t;
        jacobian->block<3,3>(0,9) = C_WS_0*dp_db_g;
        jacobian->block<3,3>(0,12) = -C_WS_0*C_doubleintegral;
        jacobian->block<3,3>(3,9) = -C_WS_0*dalpha_db_g;
        jacobian->block<3,3>(6,3) = -crossMx(C_WS_0*acc_integral);
        jacobian->block<3,3>(6,9) = C_WS_0*dv_db_g;
        jacobian->block<3,3>(6,12) = -C_WS_0*C_integral;
    }

    if (covariance) {
        assert(covariance->cols() == 15 && covariance->rows() == 15);
        //Eigen::Matrix<double,15,15> & P = *covariance;
        Eigen::Matrix<double,15,15> T = Eigen::Matrix<double,15,15>::Identity();
        T.topLeftCorner<3,3>() = C_WS_0;
        T.block<3,3>(3,3) = C_WS_0;
        T.block<3,3>(6,6) = C_WS_0;
        *covariance = T * P_delta * T.transpose();
    }
    return i;

}

int IMUImplOKVIS::error(const imuFrame &frame_i, const imuFrame &frame_j, Error_t &err, void *info)
{
    return 0;
}

int IMUImplOKVIS::repropagation()
{

}

int IMUImplOKVIS::Jacobian(const error_t &err, const imuFrame &frame_i, jacobian_t &jacobian_i, const imuFrame &frame_j, jacobian_t &jacobian_j)
{
    return 0;
}



