#include "SensorManager.hpp"
#include "KalmanFilter.hpp"
#include "PIDController.hpp"
#include "DataLogger.hpp"
#include "AITrainer.hpp"
#include <spdlog/spdlog.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>

std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

int main() {
    // FIX: SIGNTERM → SIGTERM (typo)
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    spdlog::set_level(spdlog::level::info);
    spdlog::info("Openyer starts...");

    // ── Components ────────────────────────────────────────────────────────────
    SensorManager sensors;
    sensors.start();

    KalmanFilter1D kalman(0.001, 0.03, 0.003);

    PIDController::Config pid_cfg;
    pid_cfg.kp = 2.0;
    pid_cfg.ki = 0.5;
    pid_cfg.kd = 0.1;
    PIDController pid(pid_cfg);

    // FIX: missing semicolon after DataLogger constructor call
    DataLogger logger("robot_data.csv");

    // state : [angle, gyro_z, enc_left, enc_right, distance] = 5 dims
    // action: [motor_left, motor_right]                       = 2 dims
    AITrainer ai(5, 2);

    // FIX: try/catch braces were swapped  →  try { … } catch (…) { … }
    try {
        ai.load("models/policy_net.pt");
    } catch (...) {
        spdlog::warn("Model not found – starting from scratch.");
    }

    const double setpoint_angle = 0.0; // target tilt angle (rad)
    int step = 0;

    using Clock = std::chrono::steady_clock;
    auto next         = Clock::now();
    const auto period = std::chrono::milliseconds(1); // 1 kHz control loop

    while (g_running) {
        auto data = sensors.getLatest();

        // ── Angle estimation (Kalman) ─────────────────────────────────────────
        double accel_angle = std::atan2(data.accel.x(), data.accel.z());
        double angle       = kalman.update(0.001, data.gyro.y(), accel_angle);

        // ── PID ───────────────────────────────────────────────────────────────
        double pid_output = pid.compute(setpoint_angle, angle);

        // ── AI correction (every 100 steps) ──────────────────────────────────
        std::vector<float> ai_correction = {0.0f, 0.0f};
        if (step % 100 == 0) {
            std::vector<float> state = {
                static_cast<float>(angle),
                static_cast<float>(data.gyro.z()),
                static_cast<float>(data.encoder_left),
                static_cast<float>(data.encoder_right),
                static_cast<float>(data.distance_cm)
            };
            ai_correction = ai.predict(state);
        }

        // ── Motor commands = PID + small AI correction ────────────────────────
        double left_pwm  = pid_output + 0.1 * ai_correction[0];
        // FIX: 'pid_Output' (wrong capitalisation) → 'pid_output'
        double right_pwm = pid_output + 0.1 * ai_correction[1];

        // ── Reward: closer to setpoint → higher reward ────────────────────────
        double reward = 1.0 / (1.0 + std::abs(angle - setpoint_angle));

        // ── Logging ───────────────────────────────────────────────────────────
        logger.log(data, setpoint_angle, pid_output, reward);

        // ── Store experience every 10 steps ───────────────────────────────────
        if (step % 10 == 0) {
            std::vector<float> state = {
                static_cast<float>(angle),
                static_cast<float>(data.gyro.z()),
                static_cast<float>(data.encoder_left),
                static_cast<float>(data.encoder_right),
                static_cast<float>(data.distance_cm)
            };
            AITrainer::Experience exp;
            exp.state      = state;
            exp.action     = { static_cast<float>(left_pwm),
                               static_cast<float>(right_pwm) };
            exp.reward     = static_cast<float>(reward);
            exp.next_state = state; // will be overwritten on next iteration
            // FIX: missing semicolon after addExperience call
            ai.addExperience(exp);
        }

        // ── Training step every 500 steps ────────────────────────────────────
        if (step % 500 == 0) {
            float loss = ai.trainStep(64);
            spdlog::info("Step {:6d} | angle={:.3f} | pid={:.3f} | loss={:.4f}",
                         step, angle, pid_output, loss);
        }

        // ── Checkpoint every 5000 steps ───────────────────────────────────────
        if (step % 5000 == 0 && step > 0) {
            ai.save("models/policy_net.pt");
            logger.flush();
        }

        ++step;
        next += period;
        std::this_thread::sleep_until(next);
    }

    // ── Clean shutdown ────────────────────────────────────────────────────────
    sensors.stop();
    logger.flush();
    ai.save("models/policy_net.pt");
    spdlog::info("Clean shutdown. Total steps: {}", step);
    return 0;
}
