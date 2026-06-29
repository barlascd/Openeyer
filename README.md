# Openyer

**Openyer** is an open-source self-balancing robot controller that combines a classical PID controller with a PyTorch-based behaviour-cloning neural network. The robot learns in real-time from its own PID performance and gradually improves motor control without any labelled dataset.

---

## ✨ Features

- **PID Controller** — discrete-time implementation with derivative low-pass filter and anti-windup integral clamping
- **Kalman Filter** — 1-D angle estimator that fuses MPU-6050 accelerometer and gyroscope data
- **AI Trainer** — reward-weighted behaviour cloning; the policy network learns to imitate good PID outputs and suppress bad ones
- **Sensor Manager** — 100 Hz background thread for IMU, wheel encoders, and HC-SR04 ultrasonic sensor
- **Data Logger** — lock-safe, ring-buffered CSV logger for offline analysis and model re-training
- **1 kHz control loop** — real-time-friendly main loop with `sleep_until` pacing

---

## 🏗️ Architecture

```
┌──────────────┐     SensorData      ┌────────────────┐
│ SensorManager│ ──────────────────► │ KalmanFilter1D │ → angle
└──────────────┘                     └────────────────┘
                                              │
                                              ▼
                                     ┌────────────────┐
                                     │ PIDController  │ → pid_output
                                     └────────────────┘
                                              │
                              ┌───────────────┘
                              │  (every 100 steps)
                              ▼
                     ┌────────────────┐
                     │   AITrainer    │ → ai_correction
                     │  (PolicyNet)   │
                     └────────────────┘
                              │
                              ▼
                   left_pwm = pid_output + 0.1 × ai_correction[0]
                   right_pwm = pid_output + 0.1 × ai_correction[1]
```

---

## 🛠️ Dependencies

| Library | Version | Purpose |
|---|---|---|
| [LibTorch](https://pytorch.org/cppdocs/) | ≥ 2.0 | Neural network inference & training |
| [Eigen3](https://eigen.tuxfamily.org/) | ≥ 3.3 | Matrix algebra (Kalman filter) |
| [spdlog](https://github.com/gabime/spdlog) | ≥ 1.11 | Fast structured logging |
| [pigpio](https://abyz.me.uk/rpi/pigpio/) | ≥ 79 | GPIO / PWM on Raspberry Pi |
| pthreads | POSIX | Background sensor thread |

---

## 🚀 Build & Run

### Prerequisites (Raspberry Pi / Ubuntu)

```bash
# Eigen3
sudo apt install libeigen3-dev

# spdlog
sudo apt install libspdlog-dev

# pigpio
sudo apt install libpigpio-dev pigpiod

# LibTorch – download the ARM/aarch64 build from pytorch.org
# and extract to ~/libtorch
```

### Build

```bash
git clone https://github.com/<your-username>/Openyer.git
cd Openyer
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/libtorch
make -j$(nproc)
```

### Run

```bash
mkdir -p models
sudo ./bin/openyer        # sudo required for pigpio GPIO access
```

Press **Ctrl+C** to stop. The model is saved to `models/policy_net.pt` automatically every 5 000 steps and on shutdown.

---

## 📂 Project Structure

```
Openyer/
├── include/
│   ├── SensorManager.hpp
│   ├── KalmanFilter.hpp
│   ├── PIDController.hpp
│   ├── DataLogger.hpp
│   └── AITrainer.hpp
├── src/
│   ├── main.cpp
│   ├── SensorManager.cpp
│   ├── KalmanFilter.cpp
│   ├── PIDController.cpp
│   ├── DataLogger.cpp
│   └── AITrainer.cpp
├── models/               # Saved PyTorch checkpoints (git-ignored)
├── CMakeLists.txt
├── LICENSE
└── README.md
```

---

## 🤖 How the AI Works

Openyer uses **reward-weighted behaviour cloning**:

1. Every 10 control steps, the current sensor state and motor commands are stored as an `Experience` in a replay buffer (capacity: 10 000).
2. Reward is computed as `1 / (1 + |angle − setpoint|)` — the closer to upright, the higher the reward.
3. Every 500 steps, a mini-batch of 64 experiences is sampled. The MSE loss between the network's predicted motor command and the PID's actual command is scaled by the absolute reward, so the network learns to reproduce high-quality PID behaviour more strongly.
4. Gradients are clipped at 1.0 to stabilise training.

The AI correction is blended with the PID output at **10 % weight**, ensuring the system stays stable while the network is still learning.

---

## ⚙️ Tuning

Edit the values at the top of `src/main.cpp`:

| Parameter | Default | Description |
|---|---|---|
| `pid_cfg.kp` | `2.0` | Proportional gain |
| `pid_cfg.ki` | `0.5` | Integral gain |
| `pid_cfg.kd` | `0.1` | Derivative gain |
| `setpoint_angle` | `0.0 rad` | Target tilt angle |
| AI blend factor | `0.1` | Weight of AI correction on motor output |

---

## 📊 Data Logging

Training data is written to `robot_data.csv` with the following columns:

```
timestamp_us, ax, ay, az, gx, gy, gz, enc_l, enc_r, dist, setpoint, pid_out, reward
```

Use the CSV for offline analysis, visualisation in Python/pandas, or to pre-train the policy network before deploying.

---

## 📜 License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

---

## 🙏 Acknowledgements

- [PyTorch C++ API](https://pytorch.org/cppdocs/) for making on-device neural network training practical
- [Eigen](https://eigen.tuxfamily.org/) for elegant linear algebra
- [spdlog](https://github.com/gabime/spdlog) for zero-overhead logging
