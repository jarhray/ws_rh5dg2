#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <termios.h>

namespace rh5dg2 {

// Finger order: little, ring, middle, index, thumb. Values are unscaled.
struct TouchData {
  std::array<uint16_t, 5> normal_raw{}, tangential_raw{}, direction_deg{};
  std::array<uint32_t, 5> proximity{};  // 24 significant bits.
  std::array<uint16_t, 9> palm_raw{};
};

struct SerialConfig {
  std::string port;
  int baudrate{115200};
  int hand_id{1};
  int timeout_ms{40};
  bool single_register_reads{true};
  int angle_write_ack{1};
  int stop_write_ack{1};
  int single_angle_write_ack{1};
};

// Serial/RS485 device. Failed transactions disconnect and are never retried;
// a timed-out write may have executed. Recovery requires explicit connect().
// Nonstandard angle/stop ACKs require exact register readback. Clearing the
// stop flag also always requires readback, including with the standard ACK.
class SerialDevice {
 public:
  explicit SerialDevice(SerialConfig config);
  ~SerialDevice();
  SerialDevice(const SerialDevice&) = delete;
  SerialDevice& operator=(const SerialDevice&) = delete;

  // Opening the port only configures it and discards stale input; no writes.
  void connect();
  void disconnect() noexcept;
  bool connected() const noexcept { return connected_.load(); }
  std::vector<int16_t> read_register(uint16_t address, size_t word_count);
  void write_register(uint16_t address, const std::vector<int16_t>& values);
  TouchData read_touch();

 private:
  using Bytes = std::vector<uint8_t>;
  using Clock = std::chrono::steady_clock;

  Bytes read_bytes(uint16_t address, size_t words, bool force_single = false);
  void close_locked() noexcept;
  int remaining_ms(Clock::time_point deadline) const;
  bool wait_fd(short events, Clock::time_point deadline);
  void drain_until_quiet();
  Bytes transact(uint8_t command, uint16_t address, const Bytes& data, size_t reply_size,
                 Clock::time_point group_deadline = Clock::time_point::max());

  SerialConfig config_;
  speed_t speed_;
  std::chrono::milliseconds timeout_;
  int fd_{-1};
  bool exclusive_{false};
  std::atomic<bool> connected_{false};
  std::mutex mutex_;
};

}  // namespace rh5dg2
