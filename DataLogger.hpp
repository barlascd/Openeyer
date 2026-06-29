#pragma once
#include "SensorManager.hpp"
#include <fstream>
#include <string>
#include <deque>
#include <mutex>

/**
 * @brief Thread-safe ring-buffer CSV logger.
 *
 * Buffers records in memory and flushes to disk when the buffer is full
 * or when flush() is called explicitly.
 */
class DataLogger {
public:
    struct Record {
        int64_t timestamp_us;
        double  accel_x, accel_y, accel_z;
        double  gyro_x,  gyro_y,  gyro_z;
        double  enc_left, enc_right;
        double  distance;
        double  pid_setpoint, pid_output;
        double  reward;
    };

    explicit DataLogger(const std::string& filename, size_t buffer_size = 1000);
    ~DataLogger();

    void log(const SensorData& s, double setpoint, double pid_out, double reward);

    /// Flush all buffered records to disk immediately.
    void flush();

    size_t recordCount() const;

private:
    std::ofstream          file_;
    std::deque<Record>     buffer_;
    mutable std::mutex     mutex_;
    size_t                 buffer_size_;
    size_t                 record_count_ = 0;

    void writeHeader();
    void writeRecord(const Record& r);
};
