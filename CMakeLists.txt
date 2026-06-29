cmake_minimum_required(VERSION 3.18)
project(Openyer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

# ── Dependencies ──────────────────────────────────────────────────────────────

# LibTorch (PyTorch C++ API)
find_package(Torch REQUIRED)

# FIX: was 'Eigen' → correct package name is 'Eigen3'
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# spdlog – fast structured logging
find_package(spdlog REQUIRED)

# ── Executable ───────────────────────────────────────────────────────────────

add_executable(openyer
    src/main.cpp
    src/SensorManager.cpp
    src/KalmanFilter.cpp       # FIX: was missing from original CMakeLists
    src/PIDController.cpp
    src/DataLogger.cpp
    src/AITrainer.cpp
)

target_include_directories(openyer PRIVATE include)

target_link_libraries(openyer
    ${TORCH_LIBRARIES}
    Eigen3::Eigen
    spdlog::spdlog
    pthread    # POSIX threads
    pigpio     # GPIO / PWM (Raspberry Pi)
)

# ── Compiler warnings ────────────────────────────────────────────────────────
target_compile_options(openyer PRIVATE -Wall -Wextra -Wpedantic)

# ── Output directory ─────────────────────────────────────────────────────────
set_target_properties(openyer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
