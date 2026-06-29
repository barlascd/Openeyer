#include "PIDController.hpp"
#include <algorithm>

PIDController::PIDController(const Config& cfg) : cfg_(cfg) {}

double PIDController::compute(double setpoint, double measurement) {
    auto now = Clock::now();

    if (first_call_) {
        last_time_  = now;
        first_call_ = false;
        return 0.0;
    }

    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;
    if (dt <= 0.0) return 0.0;

    double error = setpoint - measurement;

    // Proportional
    double P = cfg_.kp * error;

    // Integral with anti-windup clamp
    integral_ += error * dt;
    integral_  = std::clamp(integral_, -cfg_.integral_limit, cfg_.integral_limit);
    double I   = cfg_.ki * integral_;

    // Derivative with first-order low-pass filter
    // alpha = dt / (tau + dt)
    double alpha = dt / (cfg_.derivative_filter_tau + dt);
    double raw_d = (error - prev_error_) / dt;
    filtered_derivative_ += alpha * (raw_d - filtered_derivative_);
    double D = cfg_.kd * filtered_derivative_;

    prev_error_ = error;

    return std::clamp(P + I + D, cfg_.output_min, cfg_.output_max);
}

void PIDController::reset() {
    integral_            = 0.0;
    prev_error_          = 0.0;
    filtered_derivative_ = 0.0;
    first_call_          = true;
}

void PIDController::setConfig(const Config& cfg) {
    cfg_ = cfg;
    reset();
}
