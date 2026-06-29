#pragma once
#include <Eigen/Dense>

/**
 * @brief 1D Kalman Filter for angle estimation (angle + gyro bias states)
 *
 * State vector: x_ = [angle, gyro_bias]
 * Fuses gyroscope rate and accelerometer-derived angle.
 */
class KalmanFilter1D {
public:
    /**
     * @param q_angle  Process noise variance for angle
     * @param r_angle  Measurement noise variance for accelerometer angle
     * @param r_gyro   Measurement noise variance for gyroscope rate
     */
    KalmanFilter1D(double q_angle = 0.001,
                   double r_angle = 0.03,
                   double r_gyro  = 0.003);

    /**
     * @brief Run one predict+update cycle.
     * @param dt          Time step in seconds
     * @param gyro_rate   Raw gyroscope rate (rad/s)
     * @param accel_angle Angle derived from accelerometer (rad)
     * @return Filtered angle estimate (rad)
     */
    double update(double dt, double gyro_rate, double accel_angle);

    /// Reset state and covariance to initial values
    void reset();

private:
    Eigen::Vector2d x_;       ///< [angle, gyro_bias]
    Eigen::Matrix2d P_;       ///< Error covariance
    Eigen::Matrix2d Q_;       ///< Process noise covariance
    Eigen::Matrix2d R_diag_;  ///< Measurement noise covariance
};
