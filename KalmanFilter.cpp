#include "KalmanFilter.hpp"

KalmanFilter1D::KalmanFilter1D(double q_angle, double r_angle, double r_gyro)
{
    x_ = Eigen::Vector2d::Zero();
    P_ = Eigen::Matrix2d::Identity() * 0.1;

    Q_ << q_angle, 0.0,
          0.0,     0.003;

    R_diag_ << r_angle, 0.0,
               0.0,     r_gyro;
}

double KalmanFilter1D::update(double dt, double gyro_rate, double accel_angle)
{
    // ── Predict stage ────────────────────────────────────────────────────────
    Eigen::Matrix2d F;
    F << 1.0, -dt,
         0.0,  1.0;

    Eigen::Vector2d B(dt, 0.0);

    x_ = F * x_ + B * gyro_rate;

    // FIX: P_ must be updated with Q added, not using S (which is not yet defined).
    // Correct discrete-time covariance propagation: P = F*P*F^T + Q
    P_ = F * P_ * F.transpose() + Q_;

    // ── Update stage ─────────────────────────────────────────────────────────
    Eigen::Matrix2d H = Eigen::Matrix2d::Identity();

    // Innovation covariance  S = H*P*H^T + R
    Eigen::Matrix2d S = H * P_ * H.transpose() + R_diag_;

    // Kalman gain  K = P*H^T * S^{-1}
    Eigen::Matrix2d K = P_ * H.transpose() * S.inverse();

    Eigen::Vector2d z(accel_angle, gyro_rate);
    x_ = x_ + K * (z - H * x_);
    P_ = (Eigen::Matrix2d::Identity() - K * H) * P_;

    return x_(0); // filtered angle
}

void KalmanFilter1D::reset()
{
    x_ = Eigen::Vector2d::Zero();
    P_ = Eigen::Matrix2d::Identity() * 0.1;
}
