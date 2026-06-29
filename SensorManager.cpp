#include "SensorManager.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Platform stubs – replace with real pigpio / SPI / I²C calls on the robot.
// ─────────────────────────────────────────────────────────────────────────────

SensorManager::SensorManager() {
    latest_.accel = Eigen::Vector3d::Zero();
    latest_.gyro  = Eigen::Vector3d::Zero();
}

SensorManager::~SensorManager() {
    stop();
}

void SensorManager::start() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&SensorManager::readLoop, this);
    spdlog::info("SensorManager started");
}

void SensorManager::stop() {
    if (!running_) return;
    running_ = false;
    if (thread_.joinable()) thread_.join();
    spdlog::info("SensorManager stopped");
}

SensorData SensorManager::getLatest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

void SensorManager::readLoop() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    const auto period = milliseconds(10); // 100 Hz

    while (running_) {
        SensorData d;
        readIMU(d);
        readEncoders(d);
        readUltrasonic(d);
        d.timestamp_us = duration_cast<microseconds>(
            steady_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = d;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }
}

void SensorManager::readIMU(SensorData& d) {
    // TODO: Replace with actual MPU-6050 SPI/I2C read
    d.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
    d.gyro  = Eigen::Vector3d::Zero();
}

void SensorManager::readEncoders(SensorData& d) {
    // TODO: Replace with actual GPIO interrupt counter read
    d.encoder_left  = 0.0;
    d.encoder_right = 0.0;
}

void SensorManager::readUltrasonic(SensorData& d) {
    // TODO: Replace with actual HC-SR04 trigger/echo measurement
    d.distance_cm = 100.0;
}
