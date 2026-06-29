#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <thread>
#include <mutex>

struct SensorData {
    Eigen::Vector3d accel;          ///< Accelerometer (m/s²)
    Eigen::Vector3d gyro;           ///< Gyroscope (rad/s)
    double encoder_left  = 0.0;     ///< Left wheel encoder (rad/s)
    double encoder_right = 0.0;     ///< Right wheel encoder (rad/s)
    double distance_cm   = 0.0;     ///< Ultrasonic distance (cm)  // FIX: missing semicolon
    int64_t timestamp_us = 0;       ///< Timestamp (microseconds)
};

class SensorManager {
public:
    SensorManager();
    ~SensorManager(); // FIX: was '-SensorManager()' (minus sign instead of tilde)

    void start(); ///< Start background polling thread (100 Hz)
    void stop();

    /// Thread-safe snapshot of the latest sensor reading
    SensorData getLatest() const;

private:
    void readLoop();
    void readIMU(SensorData& d);        ///< SPI – MPU-6050
    void readEncoders(SensorData& d);   ///< GPIO interrupt counters
    void readUltrasonic(SensorData& d); ///< HC-SR04 trigger/echo

    mutable std::mutex mutex_;
    SensorData latest_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
