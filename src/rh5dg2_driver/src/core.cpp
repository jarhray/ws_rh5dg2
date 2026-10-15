#include "rh5dg2_driver/core.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace rh5dg2 {
namespace {
using Bytes = std::vector<uint8_t>;
using Clock = std::chrono::steady_clock;

bool single_angle_word(uint16_t address, size_t count) {
  return address >= 1080 && address <= 1092 && count == 1;
}

[[noreturn]] void system_error(const std::string& what) {
  const int saved_errno = errno;
  throw std::runtime_error(what + ": " + std::strerror(saved_errno));
}

void validate_span(uint16_t address, size_t count, size_t maximum) {
  if (count == 0 || count > maximum || count > 65536u - address) {
    throw std::invalid_argument("Invalid register span or protocol payload length");
  }
}

uint16_t little_word(const Bytes& bytes, size_t offset) {
  return static_cast<uint16_t>(bytes.at(offset)) |
         static_cast<uint16_t>(static_cast<uint16_t>(bytes.at(offset + 1)) << 8);
}

TouchData decode_touch(const Bytes& payload) {
  if (payload.size() != 68) {
    throw std::runtime_error("Touch reply must contain exactly 68 bytes");
  }
  TouchData result;
  for (size_t i = 0; i < 5; ++i) {
    const size_t base = i * 10;
    result.normal_raw[i] = little_word(payload, base);
    result.tangential_raw[i] = little_word(payload, base + 2);
    result.direction_deg[i] = little_word(payload, base + 4);
    result.proximity[i] = static_cast<uint32_t>(payload[base + 6]) |
                          (static_cast<uint32_t>(payload[base + 7]) << 8) |
                          (static_cast<uint32_t>(payload[base + 8]) << 16);
  }
  for (size_t i = 0; i < 9; ++i) {
    result.palm_raw[i] = little_word(payload, 50 + i * 2);
  }
  return result;
}

uint8_t checksum(const Bytes& bytes, size_t begin, size_t end) {
  uint8_t result = 0;
  for (size_t i = begin; i < end; ++i) result = static_cast<uint8_t>(result + bytes[i]);
  return result;
}

std::string hex_bytes(const Bytes& bytes) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i) out << ' ';
    out << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  return out.str();
}

speed_t baud_constant(int baudrate) {
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("Unsupported serial baudrate");
  }
}

}  // namespace

SerialDevice::SerialDevice(SerialConfig config)
    : config_(std::move(config)), speed_(baud_constant(config_.baudrate)), timeout_(config_.timeout_ms) {
  if (config_.port.empty()) throw std::invalid_argument("Serial port is empty");
  // ID 0 is broadcast and cannot identify a reply.
  if (config_.hand_id < 1 || config_.hand_id > 254)
    throw std::invalid_argument("hand_id must be 1..254 (no broadcast)");
  if (config_.timeout_ms < 1 || config_.timeout_ms > 60000)
    throw std::invalid_argument("timeout_ms must be 1..60000");
  for (const int ack : {config_.angle_write_ack, config_.stop_write_ack, config_.single_angle_write_ack})
    if (ack < 0 || ack > 255) throw std::invalid_argument("Write ACK must be a byte in 0..255");
}

SerialDevice::~SerialDevice() { disconnect(); }

void SerialDevice::connect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) return;
  fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) system_error("Opening serial port " + config_.port);
  try {
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) system_error("Serial port already owned");
    if (::ioctl(fd_, TIOCEXCL) != 0) system_error("Obtaining exclusive serial ownership");
    exclusive_ = true;
    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) system_error("Reading serial settings");
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS | CSIZE);
    settings.c_cflag |= CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, speed_) != 0 || ::cfsetospeed(&settings, speed_) != 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) != 0) {
      system_error("Configuring serial port");
    }
    if (::tcflush(fd_, TCIOFLUSH) != 0) system_error("Flushing serial port");
    // Old bytes are discarded before any new request is issued. Bound this
    // recovery step too: a continuously transmitting line cannot hang startup.
    drain_until_quiet();
    connected_.store(true);
  } catch (...) {
    close_locked();
    throw;
  }
}

void SerialDevice::disconnect() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  close_locked();
}

std::vector<int16_t> SerialDevice::read_register(uint16_t address, size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  validate_span(address, count, 126);
  const auto payload = read_bytes(address, count);
  std::vector<int16_t> words;
  words.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    const uint16_t raw = little_word(payload, i * 2);
    const int32_t signed_value = raw < 0x8000u ? raw : static_cast<int32_t>(raw) - 65536;
    words.push_back(static_cast<int16_t>(signed_value));
  }
  return words;
}

void SerialDevice::write_register(uint16_t address, const std::vector<int16_t>& values) {
  std::lock_guard<std::mutex> lock(mutex_);
  validate_span(address, values.size(), 13);
  Bytes payload;
  for (int16_t value : values) {
    const auto raw = static_cast<uint16_t>(value);
    payload.push_back(static_cast<uint8_t>(raw));
    payload.push_back(static_cast<uint8_t>(raw >> 8));
  }
  try {
    transact(0x12, address, payload, 1);
    const auto angle_ack = single_angle_word(address, values.size()) ? config_.single_angle_write_ack :
        address == 1080 && values.size() == 13 ? config_.angle_write_ack : 1;
    if (angle_ack != 1) {
      // A device-specific ACK byte is not a documented success code. Require
      // independently read, complete setpoint equality before returning success.
      // Never retry the write, including after a missing ACK or failed readback.
      const auto actual = read_bytes(address, values.size(), true);
      if (actual != payload)
        throw std::runtime_error("Angle target readback mismatch after configured ACK=" +
            std::to_string(angle_ack) + "; address=" + std::to_string(address) +
            "; expected=[" + hex_bytes(payload) +
            "]; readback=[" + hex_bytes(actual) + "]; write outcome requires inspection");
    }
    if (address == 1301 && values.size() == 1 && (values[0] == 0 || values[0] == 1) &&
        (config_.stop_write_ack != 1 || values[0] == 0)) {
      // The observed firmware retains this flag. A configured reply alone
      // cannot establish that the stop register was set. Never reset the
      // flag implicitly or retry the write; physical stop/resume is separate.
      const auto actual = read_bytes(address, 1, true);
      if (actual != payload)
        throw std::runtime_error("Stop register readback mismatch after configured ACK=" +
            std::to_string(config_.stop_write_ack) + "; expected=[" + hex_bytes(payload) + "]; readback=[" +
            hex_bytes(actual) + "]; physical stop not verified");
    }
  } catch (...) { close_locked(); throw; }
}

TouchData SerialDevice::read_touch() {
  std::lock_guard<std::mutex> lock(mutex_);
  return decode_touch(read_bytes(3000, 34));
}

Bytes SerialDevice::read_bytes(uint16_t address, size_t words, bool force_single) {
  if (!config_.single_register_reads && !force_single) return transact(0x11, address, {}, words * 2);
  // Some deployed firmware/adapter combinations reply with checksummed
  // 0xAAAA filler after the first word of block reads. Read each address
  // explicitly; never retry or silently switch modes after a failure.
  // A whole sample has an additional deadline to bound callback latency.
  const auto deadline = Clock::now() + std::max(timeout_, std::chrono::milliseconds(500));
  Bytes result;
  result.reserve(words * 2);
  for (size_t index = 0; index < words; ++index) {
    const auto part = transact(0x11, static_cast<uint16_t>(address + index), {}, 2, deadline);
    result.insert(result.end(), part.begin(), part.end());
  }
  return result;  // Publish no partial sample if one of the reads failed.
}
void SerialDevice::close_locked() noexcept {
  connected_.store(false);
  if (fd_ >= 0) {
    // Remove exclusivity explicitly; PTYs and some USB drivers retain it
    // until all other references (including the peer) have closed.
    if (exclusive_) ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    fd_ = -1;
    exclusive_ = false;
  }
}

int SerialDevice::remaining_ms(Clock::time_point deadline) const {
  const auto remaining = deadline - Clock::now();
  if (remaining <= Clock::duration::zero()) return 0;
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
  return static_cast<int>(milliseconds.count()) + 1;
}

bool SerialDevice::wait_fd(short events, Clock::time_point deadline) {
  for (;;) {
    const int remaining = remaining_ms(deadline);
    if (remaining == 0) return false;
    pollfd descriptor{fd_, events, 0};
    const int result = ::poll(&descriptor, 1, remaining);
    if (result < 0) {
      if (errno == EINTR) continue;
      system_error("Polling serial port");
    }
    if (result == 0) return false;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("Serial port disconnected or reported an I/O error");
    }
    if (descriptor.revents & events) return true;
  }
}

void SerialDevice::drain_until_quiet() {
  const auto start = Clock::now();
  const auto hard_deadline = start + timeout_ * 3;
  auto quiet_deadline = start + timeout_;
  uint8_t bytes[512];
  while (Clock::now() < hard_deadline) {
    if (!wait_fd(POLLIN, std::min(quiet_deadline, hard_deadline))) {
      if (Clock::now() >= quiet_deadline) return;
      break;
    }
    const auto count = ::read(fd_, bytes, sizeof(bytes));
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      system_error("Draining serial input");
    }
    if (count > 0) quiet_deadline = Clock::now() + timeout_;
  }
  throw std::runtime_error("Serial input did not become quiet; check adapter and other bus masters");
}

Bytes SerialDevice::transact(uint8_t command, uint16_t address, const Bytes& data, size_t reply_size,
               Clock::time_point group_deadline) {
  if (!connected_.load()) throw std::runtime_error("Device is disconnected; explicit connect required");
  Bytes request{0xEB, 0x90, static_cast<uint8_t>(config_.hand_id), static_cast<uint8_t>(command == 0x11 ? 4 : data.size() + 3),
                command, static_cast<uint8_t>(address), static_cast<uint8_t>(address >> 8)};
  if (command == 0x11) request.push_back(static_cast<uint8_t>(reply_size));
  else request.insert(request.end(), data.begin(), data.end());
  request.push_back(checksum(request, 2, request.size()));
  const auto deadline = std::min(Clock::now() + timeout_, group_deadline);
  try {
    // Any already queued response predates this transaction. No register
    // write, stop, calibration, or mode change is performed by a read call.
    if (::tcflush(fd_, TCIFLUSH) != 0) system_error("Discarding stale serial input");
    size_t sent = 0;
    while (sent < request.size()) {
      if (!wait_fd(POLLOUT, deadline)) throw std::runtime_error("Serial transaction timed out writing request");
      const auto count = ::write(fd_, request.data() + sent, request.size() - sent);
      if (count > 0) sent += static_cast<size_t>(count);
      else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        system_error("Writing serial request");
      }
    }
    Bytes incoming;
    const size_t frame_size = 8 + reply_size;
    for (;;) {
      if (!wait_fd(POLLIN, deadline)) {
        throw std::runtime_error("Serial transaction timed out waiting for a matching valid reply");
      }
      uint8_t chunk[512];
      const auto count = ::read(fd_, chunk, sizeof(chunk));
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
        system_error("Reading serial reply");
      }
      if (count == 0) continue;
      incoming.insert(incoming.end(), chunk, chunk + count);
      // Scan every possible frame start: corrupt length fields and wrong
      // headers cannot block a valid response behind them in the byte stream.
      for (size_t i = 0; i + frame_size <= incoming.size(); ++i) {
        if (incoming[i] != 0x90 || incoming[i + 1] != 0xEB ||
            incoming[i + 2] != config_.hand_id || incoming[i + 3] != reply_size + 3 ||
            incoming[i + 4] != command || incoming[i + 5] != (address & 0xff) ||
            incoming[i + 6] != (address >> 8)) continue;
        if (checksum(incoming, i + 2, i + frame_size - 1) != incoming[i + frame_size - 1]) continue;
        uint8_t expected_ack = 1;
        if (address == 1080 && data.size() == 26) expected_ack = config_.angle_write_ack;
        if (single_angle_word(address, data.size() / 2)) expected_ack = config_.single_angle_write_ack;
        if (address == 1301 && data.size() == 2 && data[0] <= 1 && data[1] == 0)
          expected_ack = config_.stop_write_ack;
        if (command == 0x12 && incoming[i + 7] != expected_ack) {
          const Bytes frame(incoming.begin() + static_cast<std::ptrdiff_t>(i),
                            incoming.begin() + static_cast<std::ptrdiff_t>(i + frame_size));
          throw std::runtime_error("Unexpected write ACK=" + std::to_string(incoming[i + 7]) +
              " (expected " + std::to_string(expected_ack) + "), address=" + std::to_string(address) +
              ", words=" + std::to_string(data.size() / 2) +
              "; motion may already have occurred; inspect readback before retrying; tx=[" +
              hex_bytes(request) + "]; rx=[" + hex_bytes(frame) + "]");
        }
        return Bytes(incoming.begin() + static_cast<std::ptrdiff_t>(i + 7),
                     incoming.begin() + static_cast<std::ptrdiff_t>(i + 7 + reply_size));
      }
      // Retain enough tail bytes to handle arbitrary packet fragmentation.
      if (incoming.size() >= frame_size) {
        incoming.erase(incoming.begin(), incoming.end() - static_cast<std::ptrdiff_t>(frame_size - 1));
      }
    }
  } catch (...) {
    // A late reply cannot accidentally satisfy the next transaction on this
    // session. Writes may already have executed and must never auto-retry.
    close_locked();
    throw;
  }
}

}  // namespace rh5dg2
