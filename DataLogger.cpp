#include "DataLogger.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>

DataLogger::DataLogger(const std::string& filename, size_t buffer_size)
    : buffer_size_(buffer_size)
{
    file_.open(filename, std::ios::out | std::ios::app);
    if (!file_.is_open())
        throw std::runtime_error("Cannot open log file: " + filename);

    // Write CSV header only when starting a fresh (empty) file
    file_.seekp(0, std::ios::end);
    if (file_.tellp() == 0) writeHeader();

    spdlog::info("DataLogger opened: {}", filename);
}

DataLogger::~DataLogger() {
    flush();
    file_.close();
}

void DataLogger::writeHeader() {
    // FIX: added missing comma after "gz" column header
    file_ << "timestamp_us,ax,ay,az,gx,gy,gz,"
          << "enc_l,enc_r,dist,setpoint,pid_out,reward\n";
}

void DataLogger::log(const SensorData& s,
                     double setpoint, double pid_out, double reward)
{
    Record r{
        s.timestamp_us,
        s.accel.x(), s.accel.y(), s.accel.z(),
        s.gyro.x(),  s.gyro.y(),  s.gyro.z(),
        s.encoder_left, s.encoder_right,
        s.distance_cm,
        setpoint, pid_out, reward
    };

    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(r);
    ++record_count_;

    // Auto-flush when buffer is full — call private helper directly
    // (no second lock needed since we already hold mutex_)
    if (buffer_.size() >= buffer_size_) {
        while (!buffer_.empty()) {
            writeRecord(buffer_.front());
            buffer_.pop_front();
        }
        file_.flush(); // FIX: was 'file_.fush()' (typo)
    }
}

void DataLogger::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    // FIX: flush() previously called itself recursively via log() path.
    // Now drains the buffer directly to avoid double-locking (deadlock).
    while (!buffer_.empty()) {
        writeRecord(buffer_.front());
        buffer_.pop_front();
    }
    file_.flush(); // FIX: was 'file_.fush()' (typo)
}

void DataLogger::writeRecord(const Record& r) {
    file_ << r.timestamp_us << ','
          << r.accel_x << ',' << r.accel_y << ',' << r.accel_z << ','
          << r.gyro_x  << ',' << r.gyro_y  << ',' << r.gyro_z  << ','
          << r.enc_left << ',' << r.enc_right << ','
          << r.distance << ','
          << r.pid_setpoint << ',' << r.pid_output << ','
          << r.reward << '\n';
}

size_t DataLogger::recordCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return record_count_;
}
