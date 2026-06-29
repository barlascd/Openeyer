#pragma once
#include <chrono>

/**
 * @brief Discrete-time PID controller with derivative filtering and anti-windup.
 */
class PIDController {
public:
    struct Config {
        double kp = 1.0;
        double ki = 0.0;
        double kd = 0.0;
        double output_min           = -1.0;  ///< Lower output clamp
        double output_max           =  1.0;  ///< Upper output clamp
        double integral_limit       = 10.0;  ///< Anti-windup clamp
        double derivative_filter_tau = 0.02; ///< Low-pass time-constant (s)
    };

    explicit PIDController(const Config& cfg);

    /// Compute PID output. Call once per control loop iteration.
    double compute(double setpoint, double measurement);

    void reset();
    void setConfig(const Config& cfg);

private:
    Config cfg_;
    double integral_           = 0.0;
    double prev_error_         = 0.0;
    double filtered_derivative_= 0.0;

    using Clock = std::chrono::steady_clock;
    Clock::time_point last_time_;
    bool first_call_ = true;
};
