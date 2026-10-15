#include "rh5dg2_driver/core.hpp"

#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace {
using Bytes = std::vector<uint8_t>;
using namespace std::chrono_literals;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <class Function> void must_throw(Function function, const std::string& message) {
  bool threw = false;
  try { function(); } catch (const std::exception&) { threw = true; }
  require(threw, message);
}

uint8_t checksum(const Bytes& bytes) {
  uint8_t sum = 0;
  for (size_t i = 2; i < bytes.size(); ++i) sum = static_cast<uint8_t>(sum + bytes[i]);
  return sum;
}

Bytes reply(uint8_t id, uint8_t command, uint16_t address, Bytes payload) {
  Bytes result{0x90, 0xEB, id, static_cast<uint8_t>(payload.size() + 3), command,
               static_cast<uint8_t>(address), static_cast<uint8_t>(address >> 8)};
  result.insert(result.end(), payload.begin(), payload.end());
  result.push_back(checksum(result));
  return result;
}

Bytes words(const std::vector<int16_t>& values) {
  Bytes result;
  for (auto value : values) {
    result.push_back(static_cast<uint8_t>(value));
    result.push_back(static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8));
  }
  return result;
}

struct Pty {
  int master{-1};
  std::string path;
  Pty() {
    master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    require(master >= 0, "posix_openpt failed");
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0 || ::ptsname(master) == nullptr) {
      ::close(master);
      throw std::runtime_error("Could not create pseudo-terminal");
    }
    path = ::ptsname(master);
  }
  ~Pty() { if (master >= 0) ::close(master); }
  Pty(const Pty&) = delete;
  Pty& operator=(const Pty&) = delete;
};

Bytes request(int master) {
  Bytes bytes;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{master, POLLIN, 0};
    const int available = ::poll(&descriptor, 1, 10);
    if (available < 0 && errno == EINTR) continue;
    if (!(descriptor.revents & POLLIN)) continue;
    uint8_t chunk[512];
    const auto count = ::read(master, chunk, sizeof(chunk));
    if (count <= 0) continue;
    bytes.insert(bytes.end(), chunk, chunk + count);
    if (bytes.size() >= 4 && bytes.size() >= 5u + bytes[3]) {
      require(bytes.size() == 5u + bytes[3], "Unexpected extra request or wrong request length");
      require(bytes[0] == 0xEB && bytes[1] == 0x90, "Invalid request header");
      const auto expected = bytes.back();
      bytes.pop_back();
      require(checksum(bytes) == expected, "Invalid request checksum");
      bytes.push_back(expected);
      return bytes;
    }
  }
  throw std::runtime_error("No request received on pseudo-terminal");
}

void send(int master, const Bytes& bytes) {
  size_t offset = 0;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (offset < bytes.size() && std::chrono::steady_clock::now() < deadline) {
    const auto count = ::write(master, bytes.data() + offset, bytes.size() - offset);
    if (count > 0) offset += static_cast<size_t>(count);
    else if (count < 0 && errno != EAGAIN && errno != EINTR) throw std::runtime_error("PTY write failed");
    else std::this_thread::sleep_for(1ms);
  }
  require(offset == bytes.size(), "PTY write timed out");
}

void check_read_request(const Bytes& bytes, uint16_t address, size_t byte_count) {
  require(bytes.size() == 9 && bytes[2] == 1 && bytes[3] == 4 && bytes[4] == 0x11 &&
          bytes[5] == (address & 255) && bytes[6] == (address >> 8) && bytes[7] == byte_count,
          "Read operation did not issue the exact expected read-only frame");
}

void scenario(const std::function<void(int)>& peer,
              const std::function<void(rh5dg2::SerialDevice&)>& exercise, int timeout_ms = 150,
              bool single_reads = false, int angle_ack = 1, int stop_ack = 1, int single_angle_ack = 1) {
  Pty pty;
  rh5dg2::SerialDevice device({pty.path, 115200, 1, timeout_ms,
                               single_reads, angle_ack, stop_ack, single_angle_ack});
  require(!device.connected(), "Construction must not connect automatically");
  device.connect();
  std::exception_ptr peer_error;
  std::thread worker([&] {
    try { peer(pty.master); } catch (...) { peer_error = std::current_exception(); }
  });
  std::exception_ptr main_error;
  try { exercise(device); } catch (...) { main_error = std::current_exception(); }
  device.disconnect();
  worker.join();
  if (main_error) std::rethrow_exception(main_error);
  if (peer_error) std::rethrow_exception(peer_error);
}

void test_resynchronization() {
  scenario([](int master) {
    check_read_request(request(master), 1136, 6);
    Bytes stream(1000, 0x44);
    // Incorrect length claiming a long frame, overlapping headers, then frames
    // with incorrect ID, command, address, length, and checksum.
    const Bytes malformed{0x90, 0x90, 0xEB, 1, 255, 0x11, 0x70, 4};
    stream.insert(stream.end(), malformed.begin(), malformed.end());
    const std::vector<Bytes> invalid{
      reply(2, 0x11, 1136, words({91, 92, 93})),
      reply(1, 0x12, 1136, words({91, 92, 93})),
      reply(1, 0x11, 1080, words({91, 92, 93})),
      reply(1, 0x11, 1136, words({91})),
    };
    for (const auto& frame : invalid) stream.insert(stream.end(), frame.begin(), frame.end());
    auto corrupt = reply(1, 0x11, 1136, words({94, 95, 96}));
    corrupt.back() ^= 0x80;
    stream.insert(stream.end(), corrupt.begin(), corrupt.end());
    send(master, stream);
    const auto good = reply(1, 0x11, 1136, words({1800, -120, -32768}));
    for (size_t i = 0; i < good.size(); ++i) {
      send(master, Bytes{good[i]});
      std::this_thread::sleep_for(1ms);
    }
  }, [](rh5dg2::SerialDevice& device) {
    require(device.read_register(1136, 3) == std::vector<int16_t>({1800, -120, -32768}),
            "Parser accepted malformed frame or lost fragmented valid reply");
    require(device.connected(), "Valid fragmented response disconnected device");
  });
}

void test_touch() {
  scenario([](int master) {
    check_read_request(request(master), 3000, 68);
    Bytes payload(68, 0);
    for (size_t finger = 0; finger < 5; ++finger) {
      payload[finger * 10] = static_cast<uint8_t>(finger + 1);
      payload[finger * 10 + 2] = 10;
      payload[finger * 10 + 4] = 90;
      payload[finger * 10 + 6] = 0x56;
      payload[finger * 10 + 7] = 0x34;
      payload[finger * 10 + 8] = 0x12;
      payload[finger * 10 + 9] = 0xAB; // Reserved high byte is not proximity.
    }
    for (size_t palm = 0; palm < 9; ++palm) {
      payload[50 + palm * 2] = static_cast<uint8_t>(palm);
      payload[51 + palm * 2] = 1;
    }
    send(master, reply(1, 0x11, 3000, payload));
  }, [](rh5dg2::SerialDevice& device) {
    const auto touch = device.read_touch();
    for (size_t i = 0; i < 5; ++i) {
      require(touch.proximity[i] == 0x123456, "24-bit touch proximity truncated or reserved bits included");
      require(touch.normal_raw[i] == i + 1 && touch.tangential_raw[i] == 10 &&
              touch.direction_deg[i] == 90, "Touch finger layout incorrect");
    }
    for (size_t i = 0; i < 9; ++i) require(touch.palm_raw[i] == 256 + i, "Touch palm layout incorrect");
  });
}

void test_short_touch_rejected() {
  scenario([](int master) {
    check_read_request(request(master), 3000, 68);
    send(master, reply(1, 0x11, 3000, Bytes(50, 0)));
  }, [](rh5dg2::SerialDevice& device) {
    must_throw([&] { device.read_touch(); }, "Incomplete touch payload accepted");
    require(!device.connected(), "Short-frame timeout did not invalidate session");
  }, 60);
}

void test_write_ack() {
  scenario([](int master) {
    const auto write = request(master);
    require(write.size() == 14 && write[4] == 0x12 && write[5] == 0x38 && write[6] == 4,
            "Write command address/size mismatch");
    require(write[7] == 0xD0 && write[8] == 7 && write[9] == 0xff && write[10] == 0xff &&
            write[11] == 0x88 && write[12] == 0xff, "Signed write data encoding mismatch");
    send(master, reply(1, 0x12, 1094, {0}));
    send(master, reply(2, 0x12, 1080, {0}));
    send(master, reply(1, 0x11, 1080, {0}));
    send(master, reply(1, 0x12, 1080, {0, 0}));
    auto corrupt = reply(1, 0x12, 1080, {1});
    corrupt.back() ^= 0x01;
    send(master, corrupt);
    send(master, reply(1, 0x12, 1080, {1}));
  }, [](rh5dg2::SerialDevice& device) {
    device.write_register(1080, {2000, -1, -120});
    require(device.connected(), "Valid write ACK disconnected session");
  });
  scenario([](int master) {
    request(master);
    send(master, reply(1, 0x12, 1080, {0}));
  }, [](rh5dg2::SerialDevice& device) {
    std::string error;
    try { device.write_register(1080, {1700}); }
    catch (const std::runtime_error& e) { error = e.what(); }
    require(error.find("ACK=0 (expected 1), address=1080, words=1") != std::string::npos,
            "Unexpected ACK lost value/address/count or was accepted");
    require(error.find("tx=[eb 90 01 05 12 38 04 a4 06 fe]") != std::string::npos &&
            error.find("rx=[90 eb 01 04 12 38 04 00 53]") != std::string::npos,
            "Unexpected ACK did not retain exact checksummed request/reply");
    require(error.find("motion may already have occurred") != std::string::npos,
            "Unexpected ACK incorrectly implies no motion occurred");
    require(!device.connected(), "Unexpected ACK left session active");
    must_throw([&] { device.write_register(1080, {1700}); }, "Unexpected ACK was retried");
  });
}

void test_device_specific_stop_ack() {
  for (const int16_t flag : {0, 1}) {
  for (const auto mode : {0, 1, 2}) {  // exact readback, wrong value, missing readback
    scenario([&](int master) {
      require(request(master) == Bytes({0xeb,0x90,0x01,0x05,0x12,0x15,0x05,
                                       static_cast<uint8_t>(flag),0x00,static_cast<uint8_t>(0x32+flag)}),
              "Stop request differs from captured physical frame");
      send(master, {0x90,0xeb,0x01,0x04,0x12,0x15,0x05,0x02,0x33});
      check_read_request(request(master), 1301, 2);
      if (mode == 2) return;
      send(master, reply(1, 0x11, 1301, words({static_cast<int16_t>(mode == 0 ? flag : 1-flag)})));
      if (mode != 0) return;
      // The next operation must work on the same session, without reconnect.
      check_read_request(request(master), 1140, 2);
      send(master, reply(1, 0x11, 1140, words({1198})));
      const auto write = request(master);
      require(write[4] == 0x12 && write[5] == 0x3c && write[6] == 4,
              "Subsequent angle command used the wrong register");
      send(master, reply(1, 0x12, 1084, {1}));
    }, [&](rh5dg2::SerialDevice& device) {
      if (mode == 0) {
        device.write_register(1301, {flag});
        require(device.connected(), "Verified stop reply disconnected session");
        require(device.read_register(1140, 1) == std::vector<int16_t>({1198}),
                "Feedback failed after configured stop reply");
        device.write_register(1084, {1300});
      } else {
        must_throw([&] { device.write_register(1301, {flag}); }, "Unverified stop register accepted");
        require(!device.connected(), "Stop readback failure left session active");
        must_throw([&] { device.write_register(1301, {flag}); }, "Failed stop verification allowed retry");
      }
    }, 70, false, 32, 2);
  }
  }
  // Configured stop ACK must not leak into other registers or invalid flag values.
  for (const uint16_t address : {1003, 1084, 1301}) {
    scenario([&](int master) {
      request(master);
      send(master, reply(1, 0x12, address, {2}));
    }, [&](rh5dg2::SerialDevice& device) {
      must_throw([&] { device.write_register(address, {2}); }, "Stop ACK policy leaked");
      require(!device.connected(), "Unexpected ACK left connection open");
    }, 70, false, 32, 2);
  }
  for (const auto stop_ack : {1, 2}) {
    scenario([&](int master) {
      request(master);
      send(master, reply(1, 0x12, 1301, {static_cast<uint8_t>(stop_ack == 1 ? 2 : 3)}));
    }, [](rh5dg2::SerialDevice& device) {
      must_throw([&] { device.write_register(1301, {1}); }, "Unconfigured stop ACK accepted");
      require(!device.connected(), "Unknown stop ACK left connection open");
    }, 70, false, 32, stop_ack);
  }
}

void test_single_angle_ack() {
  for (const uint16_t address : {1080, 1084, 1092}) {
    for (const auto mode : {0, 1, 2, 3}) {  // match, bad readback, missing readback, wrong ACK
      scenario([&](int master) {
        const auto write = request(master);
        require(write.size() == 10 && write[4] == 0x12 && write[5] == (address & 255) &&
                write[6] == (address >> 8) && write[7] == 0x08 && write[8] == 0x07,
                "Single angle write did not address exactly one requested register");
        send(master, reply(1, 0x12, address, {static_cast<uint8_t>(mode == 3 ? 32 : 2)}));
        if (mode == 3) return;
        check_read_request(request(master), address, 2);
        if (mode == 2) return;
        send(master, reply(1, 0x11, address, words({static_cast<int16_t>(mode == 1 ? 1801 : 1800)})));
      }, [&](rh5dg2::SerialDevice& device) {
        if (mode == 0) {
          device.write_register(address, {1800});
          require(device.connected(), "Verified single angle reply disconnected");
        } else {
          must_throw([&] { device.write_register(address, {1800}); }, "Unverified single target accepted");
          require(!device.connected(), "Single angle failure left session active");
          must_throw([&] { device.write_register(address, {1800}); }, "Single angle failure retried");
        }
      }, 60, false, 32, 1, 2);
    }
  }
  for (const uint16_t address : {1079, 1093, 1003, 1301}) {
    scenario([&](int master) {
      request(master);
      send(master, reply(1, 0x12, address, {2}));
    }, [&](rh5dg2::SerialDevice& device) {
      must_throw([&] { device.write_register(address, {1}); }, "Single angle ACK leaked outside angle bank");
    }, 60, false, 32, 1, 2);
  }
}

void test_write_timeout_and_no_retry() {
  scenario([](int master) {
    const auto write = request(master);
    require(write[4] == 0x12, "Expected write request");
    auto corrupt = reply(1, 0x12, 1080, {1});
    corrupt.back() ^= 0x01;
    send(master, corrupt);
    std::this_thread::sleep_for(90ms);
    uint8_t bytes[128];
    const auto count = ::read(master, bytes, sizeof(bytes));
    require(count <= 0, "Timed-out write was automatically retried");
    // A late correct ACK must not make the failed session usable again.
    send(master, reply(1, 0x12, 1080, {1}));
  }, [](rh5dg2::SerialDevice& device) {
    const auto started = std::chrono::steady_clock::now();
    must_throw([&] { device.write_register(1080, {1700}); }, "Bad checksum ACK accepted");
    const auto elapsed = std::chrono::steady_clock::now() - started;
    require(elapsed >= 40ms && elapsed < 700ms, "Transaction timeout was not bounded");
    require(!device.connected(), "Timeout did not disconnect session");
    must_throw([&] { device.write_register(1080, {1700}); }, "Disconnected write retried without reconnect");
    must_throw([&] { device.read_register(1136, 13); }, "Disconnected read restarted communication");
  }, 60);
}

void test_device_specific_angle_ack() {
  const std::vector<int16_t> targets{1800,1800,1800,0,1800,0,1850,1850,1850,1850,1700,1600,2040};
  for (const auto mode : {0, 1, 2}) {  // exact match, mismatch, missing readback
    scenario([&](int master) {
      const auto write = request(master);
      require(write.size() == 34 && write[4] == 0x12, "Expected one complete angle write");
      // Captured physical response on 2026-10-13, including its checksum.
      send(master, {0x90,0xeb,0x01,0x04,0x12,0x38,0x04,0x20,0x73});
      for (size_t i = 0; i < targets.size(); ++i) {
        check_read_request(request(master), static_cast<uint16_t>(1080+i), 2);
        if (mode == 2 && i == 4) return;
        const int16_t value = static_cast<int16_t>(targets[i] + (mode == 1 && i == 4 ? 1 : 0));
        send(master, reply(1, 0x11, static_cast<uint16_t>(1080+i), words({value})));
      }
    }, [&](rh5dg2::SerialDevice& device) {
      if (mode == 0) {
        device.write_register(1080, targets);
        require(device.connected(), "Verified device-specific ACK disconnected");
      } else {
        must_throw([&] { device.write_register(1080, targets); }, "Unverified device-specific ACK accepted");
        require(!device.connected(), "Failed verification left session active");
        must_throw([&] { device.write_register(1080, targets); }, "Failed verification allowed retry");
      }
    }, 70, false, 32);  // Readback must be single-word even in block feedback mode.
  }
  scenario([](int master) {
    request(master);
    send(master, reply(1, 0x12, 1080, {33}));
  }, [&](rh5dg2::SerialDevice& device) {
    must_throw([&] { device.write_register(1080, targets); }, "Unconfigured ACK accepted");
    require(!device.connected(), "Wrong ACK did not close serial");
  }, 70, true, 32);
  scenario([](int master) {
    request(master);
    send(master, reply(1, 0x12, 1301, {32}));
  }, [](rh5dg2::SerialDevice& device) {
    must_throw([&] { device.write_register(1301, {1}); }, "Angle-only ACK policy leaked into stop");
  }, 70, true, 32);
}

void test_exclusive_and_reconnect() {
  Pty pty;
  rh5dg2::SerialDevice owner({pty.path, 115200, 1, 30});
  owner.connect();
  rh5dg2::SerialDevice competitor({pty.path, 115200, 1, 30});
  must_throw([&] { competitor.connect(); }, "Two devices acquired same serial port");
  require(owner.connected(), "Competitor invalidated owner's connection");
  owner.disconnect();
  // Reopening is explicit and drains a stale queued response first.
  send(pty.master, reply(1, 0x11, 1136, words({999})));
  competitor.connect();
  std::exception_ptr peer_error;
  std::thread worker([&] {
    try {
      check_read_request(request(pty.master), 1136, 2);
      send(pty.master, reply(1, 0x11, 1136, words({1600})));
    } catch (...) { peer_error = std::current_exception(); }
  });
  std::exception_ptr main_error;
  try { require(competitor.read_register(1136, 1)[0] == 1600, "Reconnect consumed stale pre-request reply"); }
  catch (...) { main_error = std::current_exception(); }
  competitor.disconnect();
  worker.join();
  if (main_error) std::rethrow_exception(main_error);
  if (peer_error) std::rethrow_exception(peer_error);
}

void test_incomplete_reply_and_noisy_timeout() {
  scenario([](int master) {
    check_read_request(request(master), 1136, 26);
    auto incomplete = reply(1, 0x11, 1136, words(std::vector<int16_t>(13, 1500)));
    incomplete.pop_back();
    send(master, incomplete);
  }, [](rh5dg2::SerialDevice& device) {
    must_throw([&] { device.read_register(1136, 13); }, "Incomplete matching frame accepted");
    require(!device.connected(), "Incomplete frame did not invalidate session");
  }, 60);
  scenario([](int master) {
    check_read_request(request(master), 1136, 26);
    for (int i = 0; i < 20; ++i) {
      send(master, Bytes(128, 0x90));
      std::this_thread::sleep_for(5ms);
    }
  }, [](rh5dg2::SerialDevice& device) {
    const auto start = std::chrono::steady_clock::now();
    must_throw([&] { device.read_register(1136, 13); }, "Continuous noise accepted as a response");
    const auto elapsed = std::chrono::steady_clock::now() - start;
    require(elapsed >= 40ms && elapsed < 700ms, "Incoming noise extended transaction deadline");
    require(!device.connected(), "Noisy timeout left session active");
  }, 60);
}

void test_disconnected_peer() {
  Pty pty;
  rh5dg2::SerialDevice device({pty.path, 115200, 1, 100});
  device.connect();
  ::close(pty.master);
  pty.master = -1;
  const auto started = std::chrono::steady_clock::now();
  must_throw([&] { device.read_register(1136, 13); }, "Disconnected serial peer did not raise I/O error");
  require(!device.connected(), "Disconnected peer left device connected");
  require(std::chrono::steady_clock::now() - started < 700ms, "Disconnected peer caused unbounded wait");
}

void test_single_register_reads() {
  const std::vector<int16_t> angles{1648,1650,1652,0,1651,-1,1857,1853,1850,1852,1702,1582,2046};
  Bytes touch(68, 0);
  for (size_t i = 0; i < 5; ++i) {
    touch[10*i] = 17; touch[10*i+2] = 21; touch[10*i+4] = 90;
    touch[10*i+6] = 0x56; touch[10*i+7] = 0x34; touch[10*i+8] = 0x12;
  }
  scenario([&](int master) {
    for (uint16_t i = 0; i < 13; ++i) {
      check_read_request(request(master), 1136+i, 2);
      send(master, reply(1, 0x11, 1136+i, words({angles[i]})));
    }
    for (uint16_t i = 0; i < 34; ++i) {
      check_read_request(request(master), 3000+i, 2);
      send(master, reply(1, 0x11, 3000+i, Bytes{touch[2*i], touch[2*i+1]}));
    }
  }, [&](rh5dg2::SerialDevice &device) {
    require(device.read_register(1136, 13) == angles, "Single reads changed register order or signed data");
    const auto data = device.read_touch();
    for (size_t i = 0; i < 5; ++i)
      require(data.proximity[i] == 0x123456 && data.normal_raw[i] == 17 &&
              data.tangential_raw[i] == 21 && data.direction_deg[i] == 90,
              "Single-register touch assembly lost the 24-bit layout");
  }, 100, true);
}

void test_single_register_partial_failure() {
  scenario([](int master) {
    for (uint16_t i = 0; i < 3; ++i) {
      check_read_request(request(master), 1136+i, 2);
      send(master, reply(1, 0x11, 1136+i, words({1600})));
    }
    check_read_request(request(master), 1139, 2);
    // No reply to channel 3: no partial group or later channel may be returned.
    std::this_thread::sleep_for(90ms);
  }, [](rh5dg2::SerialDevice &device) {
    must_throw([&] { device.read_register(1136, 13); }, "Partial group was returned as feedback");
    require(!device.connected(), "Partial read did not disconnect");
    must_throw([&] { device.read_register(1136, 13); }, "Partial read was retried automatically");
  }, 40, true);
}

void test_invalid_arguments() {
  must_throw([] { rh5dg2::SerialDevice({""}); }, "Empty port accepted");
  must_throw([] { rh5dg2::SerialDevice({"/not-opened", 115200, 0, 100}); }, "Broadcast ID accepted");
  must_throw([] { rh5dg2::SerialDevice({"/not-opened", 115200, 255, 100}); }, "Invalid hand ID accepted");
  must_throw([] { rh5dg2::SerialDevice({"/not-opened", 115200, 1, 0}); }, "Zero timeout accepted");
  must_throw([] { rh5dg2::SerialDevice({"/not-opened", 123, 1, 100}); }, "Unsupported baudrate accepted");
  for (const auto ack : {-1, 256}) {
    must_throw([&] { rh5dg2::SerialDevice({"/not-opened", 115200, 1, 100, false, ack}); },
               "Invalid block angle ACK accepted");
    must_throw([&] { rh5dg2::SerialDevice({"/not-opened", 115200, 1, 100, false, 1, ack}); },
               "Invalid stop ACK accepted");
    must_throw([&] { rh5dg2::SerialDevice({"/not-opened", 115200, 1, 100, false, 1, 1, ack}); },
               "Invalid single angle ACK accepted");
  }
  Pty pty;
  rh5dg2::SerialDevice device({pty.path, 115200, 1, 100});
  device.connect();
  must_throw([&] { device.read_register(1136, 0); }, "Zero-length read accepted");
  must_throw([&] { device.read_register(1136, 127); }, "Oversized read accepted");
  must_throw([&] { device.read_register(65535, 2); }, "Register address overflow accepted");
  must_throw([&] { device.write_register(1080, {}); }, "Empty write accepted");
  must_throw([&] { device.write_register(1080, std::vector<int16_t>(14)); }, "Oversized write accepted");
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests{
    {"stream resynchronization", test_resynchronization},
    {"24-bit touch", test_touch}, {"short touch rejected", test_short_touch_rejected},
    {"write ACK validation", test_write_ack}, {"device-specific angle ACK with readback", test_device_specific_angle_ack},
    {"device-specific stop ACK with readback", test_device_specific_stop_ack},
    {"single-angle ACK scope and readback", test_single_angle_ack},
    {"timeout and no retry", test_write_timeout_and_no_retry},
    {"exclusive port and reconnect", test_exclusive_and_reconnect},
    {"incomplete frame and noisy deadline", test_incomplete_reply_and_noisy_timeout},
    {"disconnected peer", test_disconnected_peer}, {"argument validation", test_invalid_arguments},
    {"single-register feedback and touch", test_single_register_reads},
    {"single-register partial failure", test_single_register_partial_failure}
  };
  size_t failures = 0;
  for (const auto& test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  if (failures) std::cerr << failures << " test group(s) failed\n";
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
