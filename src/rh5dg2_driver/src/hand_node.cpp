#include "rh5dg2_driver/core.hpp"
#include "rh5dg2_driver/registers.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rh5dg2_interfaces/msg/hand_command.hpp>
#include <rh5dg2_interfaces/msg/joint_command.hpp>
#include <rh5dg2_interfaces/msg/joint_feedback.hpp>
#include <rh5dg2_interfaces/msg/command_result.hpp>
#include <rh5dg2_interfaces/msg/hand_state.hpp>
#include <rh5dg2_interfaces/msg/touch_state.hpp>
#include <rh5dg2_interfaces/srv/set_values.hpp>
#include <rh5dg2_interfaces/srv/get_state.hpp>
#include <rh5dg2_interfaces/srv/set_enabled.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rh5dg2 {
using Steady = std::chrono::steady_clock;
using State = rh5dg2_interfaces::msg::HandState;
using Touch = rh5dg2_interfaces::msg::TouchState;
using Command = rh5dg2_interfaces::msg::HandCommand;
using JointCommand = rh5dg2_interfaces::msg::JointCommand;
using Feedback = rh5dg2_interfaces::msg::JointFeedback;
using Result = rh5dg2_interfaces::msg::CommandResult;
using SetValues = rh5dg2_interfaces::srv::SetValues;
using GetState = rh5dg2_interfaces::srv::GetState;
using SetEnabled = rh5dg2_interfaces::srv::SetEnabled;
using Trigger = std_srvs::srv::Trigger;

class HandNode : public rclcpp::Node {
 public:
  HandNode() : Node("driver") {
    const auto side = parameter<std::string>("side", "left");
    if (side != "left" && side != "right") throw std::invalid_argument("side must be left or right");
    SerialConfig serial;
    serial.port = parameter<std::string>("port", "/dev/rh5dg2_" + side);
    serial.baudrate = integer_parameter("baudrate", 115200, 9600, 921600);
    serial.hand_id = integer_parameter("hand_id", 1, 1, 254);
    serial.timeout_ms = integer_parameter("serial_timeout_ms", 40, 1, 1000);
    serial.angle_write_ack = integer_parameter("angle_write_ack", 1, 0, 255);
    serial.stop_write_ack = integer_parameter("stop_write_ack", 1, 0, 255);
    serial.single_angle_write_ack = integer_parameter("single_angle_write_ack", 1, 0, 255);
    startup_enable_ = parameter<bool>("enable_write", true);
    command_max_age_ = parameter<double>("command_max_age_sec", 0.5);
    if (!std::isfinite(command_max_age_) || command_max_age_ <= 0)
      throw std::invalid_argument("command_max_age_sec must be positive and finite");
    const double slow = rate("slow_rate", 1.0);
    rates_ = {rate("angle_rate", 5.0), rate("force_rate", 2.0, true),
              rate("current_rate", 2.0, true), slow, slow, slow, rate("touch_rate", 1.0, true)};
    const double state_rate = rate("state_rate", 20.0);
    state_.joint_names = joint_names(side);
    state_.force_sensor_present.fill(true);
    std::fill(state_.force_sensor_present.begin() + 6, state_.force_sensor_present.begin() + 10, false);
    state_.header.frame_id = touch_.header.frame_id = side + "_hand_base";
    configure_joint_states();
    device_ = std::make_unique<SerialDevice>(serial);
    try {
      device_->connect();
    } catch (const std::exception &e) {
      throw std::runtime_error("Cannot connect to hand serial port '" + serial.port + "': " + e.what());
    }

    state_pub_ = create_publisher<State>("state", 5);
    touch_pub_ = create_publisher<Touch>("touch", rclcpp::SensorDataQoS());
    result_pub_ = create_publisher<Result>("command_result", 10);
    const std::array<const char *, 6> feedback_topics{
        "angle_actual", "force_actual", "current_actual", "temperature", "motor_status", "error_code"};
    for (size_t i = 0; i < feedback_topics.size(); ++i)
      feedback_pubs_[i] = create_publisher<Feedback>(feedback_topics[i], rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<Command>("command", rclcpp::QoS(1),
        [this](Command::ConstSharedPtr msg) { topic_command(msg->kind, *msg); });
    const std::array<const char *, 5> command_topics{
        "angle_set", "speed_set", "force_set", "current_limit_set", "mode_set"};
    for (size_t i = 0; i < command_topics.size(); ++i)
      joint_command_subs_[i] = create_subscription<JointCommand>(command_topics[i], rclcpp::QoS(1),
          [this, i](JointCommand::ConstSharedPtr msg) { topic_command(static_cast<uint8_t>(i), *msg); });
    set_srv_ = create_service<SetValues>("set_values", [this](
        SetValues::Request::SharedPtr req, SetValues::Response::SharedPtr res) {
      *res = execute(req->kind, req->values);
    });
    get_srv_ = create_service<GetState>("get_state", [this](
        GetState::Request::SharedPtr, GetState::Response::SharedPtr res) {
      res->state = snapshot();
      res->success = !res->state.stale;
      res->message = res->state.status;
    });
    enable_srv_ = create_service<SetEnabled>("set_enabled", [this](
        SetEnabled::Request::SharedPtr req, SetEnabled::Response::SharedPtr res) {
      res->success = set_enabled(req->enabled, res->message);
    });
    stop_srv_ = create_service<Trigger>("stop", [this](
        Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { res->success = set_stop(true, res->message); });
    clear_stop_srv_ = create_service<Trigger>("clear_stop", [this](
        Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { res->success = set_stop(false, res->message); });
    clear_srv_ = create_service<Trigger>("clear_errors", [this](
        Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { *res = clear_errors(); });
    reconnect_srv_ = create_service<Trigger>("reconnect", [this](
        Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { *res = reconnect(); });
    publish_all_feedback();
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { poll_one(); });
    state_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / state_rate), [this] { publish_state(); });
    RCLCPP_INFO(get_logger(), "%s hand on %s, ID %d; command input %s",
                side.c_str(), serial.port.c_str(), serial.hand_id,
                startup_enable_ ? "opens after initial feedback" : "disabled");
  }

 private:
  template<typename T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  int integer_parameter(const std::string &name, int64_t value, int64_t lo, int64_t hi) {
    const auto result = parameter<int64_t>(name, value);
    if (result < lo || result > hi) throw std::invalid_argument(name + " outside allowed range");
    return static_cast<int>(result);
  }
  double rate(const std::string &name, double value, bool optional = false) {
    const auto result = parameter<double>(name, value);
    if (!std::isfinite(result) || result > 100 || (result < 0.1 && !(optional && result == 0)))
      throw std::invalid_argument(name + " must be in [0.1, 100] Hz" + (optional ? ", or 0 to disable" : ""));
    return result;
  }
  void configure_joint_states() {
    const bool publish = parameter<bool>("publish_joint_states", false);
    const bool confirmed = parameter<bool>("joint_mapping_confirmed", false);
    if (!publish) return;
    if (!confirmed) throw std::invalid_argument("joint_states require joint_mapping_confirmed=true");
    const auto zero = reference_zero();
    zeros_ = parameter<std::vector<double>>("joint_zero_raw", {zero.begin(), zero.end()});
    directions_ = parameter<std::vector<double>>("joint_direction", {-1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,-1});
    if (zeros_.size() != 13 || directions_.size() != 13)
      throw std::invalid_argument("joint_zero_raw and joint_direction must have 13 entries");
    for (size_t i = 0; i < 13; ++i)
      if (!std::isfinite(zeros_[i]) || (directions_[i] != 1 && directions_[i] != -1))
        throw std::invalid_argument("Joint zeros must be finite, directions must be +1 or -1");
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 5);
  }
  bool fresh(size_t i) const {
    return device_->connected() && samples_[i].acquired && rates_[i] > 0 &&
        std::chrono::duration<double>(Steady::now() - samples_[i].at).count() <= 3.0 / rates_[i] + 0.2;
  }
  bool ready() const { return fresh(0) && fresh(5); }
  bool no_errors() const {
    return std::all_of(state_.error_code.begin(), state_.error_code.end(), [](int16_t x) { return x == 0; });
  }
  State snapshot() {
    state_.header.stamp = now();
    state_.connected = device_->connected();
    state_.enabled = enabled_;
    state_.angle_valid = fresh(0);
    state_.force_valid = fresh(1);
    state_.current_valid = fresh(2);
    state_.temperature_valid = fresh(3);
    state_.motor_status_valid = fresh(4);
    state_.error_valid = fresh(5);
    state_.stale = !state_.angle_valid || !state_.error_valid;
    if (!state_.connected) state_.status = fault_message_;
    else if (state_.stale) state_.status = "Waiting for fresh angle and error feedback";
    else if (!no_errors()) state_.status = "Device reports motor errors";
    else state_.status = enabled_ ? "Commands enabled" : "Commands disabled";
    return state_;
  }
  void invalidate_feedback() {
    for (auto &sample : samples_) sample.acquired = false;
    touch_.valid = false;
    touch_pub_->publish(touch_);
    publish_all_feedback();
  }
  void fault(const std::string &message) {
    enabled_ = startup_enable_ = false;
    device_->disconnect();
    fault_message_ = "Communication fault; explicit reconnect required: " + message;
    RCLCPP_ERROR(get_logger(), "%s", fault_message_.c_str());
    invalidate_feedback();
  }
  bool set_enabled(bool desired, std::string &message) {
    startup_enable_ = false;
    if (!desired) {
      enabled_ = false;
      message = "Commands disabled; device targets unchanged";
      return true;
    }
    if (!device_->connected()) { message = "Device disconnected"; return false; }
    if (!ready()) { message = "Fresh angle and error feedback required"; return false; }
    if (!no_errors()) { message = "Device reports errors; clear them first"; return false; }
    if (!enabled_) enabled_after_ns_ = now().nanoseconds();
    enabled_ = true;
    message = "Commands enabled; emergency stop unchanged";
    return true;
  }
  // Only explicit stop/clear_stop services write the emergency-stop register.
  bool set_stop(bool stopped, std::string &message) {
    enabled_ = startup_enable_ = false;
    try {
      device_->write_register(1301, {static_cast<int16_t>(stopped)});
      message = stopped ? "Emergency stop acknowledged; commands disabled" :
                          "Emergency stop cleared; commands disabled";
      return true;
    } catch (const std::exception &e) { fault(e.what()); message = fault_message_; return false; }
  }
  Trigger::Response clear_errors() {
    Trigger::Response response;
    try {
      device_->write_register(1003, {1});
      samples_[5].acquired = false;
      samples_[5].due = Steady::now();
      publish_feedback(5);
      response.success = true;
      response.message = "Clear-errors acknowledged; waiting for fresh error feedback";
    } catch (const std::exception &e) { fault(e.what()); response.message = fault_message_; }
    return response;
  }
  Trigger::Response reconnect() {
    Trigger::Response response;
    if (enabled_) { response.message = "Stop or disable before reconnecting"; return response; }
    startup_enable_ = false;
    device_->disconnect();
    invalidate_feedback();
    try {
      device_->connect();
      for (auto &sample : samples_) sample.due = Steady::now();
      response.success = true;
      response.message = "Reconnected; commands disabled";
    } catch (const std::exception &e) { fault(e.what()); response.message = fault_message_; }
    return response;
  }
  // Skip -1 registers outright: stored targets may be outside the manual limits.
  void write_command(const PreparedCommand &command, bool partial) {
    if (!partial) {
      device_->write_register(command.address, {command.values.begin(), command.values.end()});
      return;
    }
    size_t completed = 0;
    for (size_t i = 0; i < command.values.size(); ++i) {
      if (command.values[i] == -1) continue;
      try { device_->write_register(static_cast<uint16_t>(command.address + i), {command.values[i]}); }
      catch (const std::exception &e) {
        throw std::runtime_error("Angle write failed at channel " + std::to_string(i) + " after " +
            std::to_string(completed) + " completed writes; failed write may have executed; " +
            "remaining channels not sent: " + e.what());
      }
      ++completed;
    }
  }
  SetValues::Response execute(uint8_t kind, const JointValues &requested) {
    SetValues::Response response;
    if (!enabled_ || !device_->connected()) {
      response.message = "Commands disabled or device disconnected";
      return response;
    }
    if (!ready() || !no_errors()) {
      enabled_ = false;
      response.message = "Feedback stale or device error; commands disabled";
      return response;
    }
    try {
      const auto command = prepare_command(kind, requested);
      const bool partial = kind == 0 && std::find(requested.begin(), requested.end(), -1) != requested.end();
      write_command(command, partial);
      response.success = true;
      response.applied_values = command.values;
      response.clamped = command.clamped;
      const bool clamped = std::any_of(command.clamped.begin(), command.clamped.end(), [](bool v) { return v; });
      response.message = clamped ? "Targets clamped to configured limits; write acknowledged" : "Command completed";
      if (kind == 0) samples_[0].due = Steady::now();
    } catch (const std::invalid_argument &e) {
      response.message = e.what();
    } catch (const std::exception &e) {
      fault(e.what());
      response.message = fault_message_;
    }
    return response;
  }
  template<typename Message> void topic_command(uint8_t kind, const Message &command) {
    Result response;
    response.header.stamp = now();
    response.command_id = command.command_id;
    const auto &stamp = command.header.stamp;
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const double age = (now().nanoseconds() - stamp_ns) / 1e9;
    if (stamp_ns <= 0 || stamp.nanosec >= 1000000000 || age > command_max_age_ || age < -0.1)
      response.message = "Command timestamp missing, expired, or in the future";
    else if (enabled_ && stamp_ns < enabled_after_ns_)
      response.message = "Command timestamp predates the current enable";
    else {
      const auto result = execute(kind, command.values);
      response.accepted = result.success;
      response.message = result.message;
      response.applied_values = result.applied_values;
      response.clamped = result.clamped;
    }
    result_pub_->publish(response);
  }
  void poll_one() {
    if (!device_->connected()) return;
    for (size_t scanned = 0; scanned < samples_.size(); ++scanned) {
      const size_t i = cursor_++ % samples_.size();
      auto &sample = samples_[i];
      if (rates_[i] == 0 || Steady::now() < sample.due) continue;
      try {
        if (i == 6) poll_touch();
        else {
          static constexpr std::array<uint16_t, 6> addresses{1136,1150,1164,1206,1192,1178};
          const auto values = device_->read_register(addresses[i], 13);
          if (values.size() != 13 || is_placeholder_feedback(values) ||
              (i == 0 && is_placeholder_angle_feedback(values)))
            throw std::runtime_error("Invalid feedback at register " + std::to_string(addresses[i]));
          std::copy(values.begin(), values.end(), feedback_values(i).begin());
          feedback_stamp(i) = now();
        }
        sample.at = Steady::now();
        sample.acquired = true;
        sample.due = sample.at + std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(1.0 / rates_[i]));
        if (i < feedback_pubs_.size()) publish_feedback(i);
        if (i == 0) publish_joint_sample();
        if (i == 5 && !no_errors()) enabled_ = false;
        if (startup_enable_ && ready()) {
          std::string message;
          if (!set_enabled(true, message)) RCLCPP_WARN(get_logger(), "Enable rejected: %s", message.c_str());
        }
      } catch (const std::exception &e) { fault(e.what()); }
      return;  // One sample per callback; serial work stays on this executor thread.
    }
  }
  void poll_touch() {
    const auto data = device_->read_touch();
    touch_.header.stamp = now();
    touch_.valid = true;
    for (size_t i = 0; i < 5; ++i) {
      touch_.normal_force_n[i] = data.normal_raw[i] * 0.01;
      touch_.tangential_force_n[i] = data.tangential_raw[i] * 0.01;
      touch_.direction_deg[i] = data.direction_deg[i];
      touch_.direction_valid[i] = data.direction_deg[i] <= 359 && data.tangential_raw[i] != 0xffff;
      touch_.proximity[i] = data.proximity[i];
    }
    touch_.palm_raw = data.palm_raw;
    touch_pub_->publish(touch_);
  }
  void publish_joint_sample() {
    if (!joint_pub_) return;
    sensor_msgs::msg::JointState message;
    message.header = state_.header;
    message.header.stamp = state_.angle_stamp;
    message.name.assign(state_.joint_names.begin(), state_.joint_names.end());
    for (size_t i = 0; i < 13; ++i)
      message.position.push_back((state_.angle_raw[i] - zeros_[i]) * directions_[i] * 3.14159265358979323846 / 1800.0);
    joint_pub_->publish(message);
  }
  JointValues &feedback_values(size_t i) {
    const std::array<JointValues *, 6> fields{&state_.angle_raw, &state_.force_raw, &state_.current_ma,
        &state_.temperature_c, &state_.motor_status, &state_.error_code};
    return *fields[i];
  }
  builtin_interfaces::msg::Time &feedback_stamp(size_t i) {
    const std::array<builtin_interfaces::msg::Time *, 6> fields{
        &state_.angle_stamp, &state_.force_stamp, &state_.current_stamp,
        &state_.temperature_stamp, &state_.motor_status_stamp, &state_.error_stamp};
    return *fields[i];
  }
  void publish_feedback(size_t i) {
    static constexpr std::array<const char *, 6> units{"0.1 deg", "g", "mA", "degC", "code", "code"};
    Feedback message;
    message.header.frame_id = state_.header.frame_id;
    message.header.stamp = feedback_stamp(i);
    message.valid = fresh(i);
    message.unit = units[i];
    message.joint_names = state_.joint_names;
    message.values = feedback_values(i);
    message.channel_present.fill(true);
    if (i == 1) message.channel_present = state_.force_sensor_present;
    feedback_pubs_[i]->publish(message);
    samples_[i].published_valid = message.valid;
  }
  void publish_all_feedback() {
    for (size_t i = 0; i < feedback_pubs_.size(); ++i) publish_feedback(i);
  }
  void publish_state() {
    if (!ready()) enabled_ = false;
    ++state_.sequence;
    state_pub_->publish(snapshot());
    for (size_t i = 0; i < feedback_pubs_.size(); ++i)
      if (samples_[i].published_valid != fresh(i)) publish_feedback(i);
    if (touch_.valid && !fresh(6)) { touch_.valid = false; touch_pub_->publish(touch_); }
  }

  struct Sample {
    Steady::time_point at{}, due{};
    bool acquired{false}, published_valid{false};
  };
  std::unique_ptr<SerialDevice> device_;
  bool enabled_{false}, startup_enable_{false};
  double command_max_age_{0.5};
  int64_t enabled_after_ns_{0};
  std::string fault_message_;
  std::vector<double> zeros_, directions_;
  State state_;
  Touch touch_;
  std::array<double, 7> rates_{};
  std::array<Sample, 7> samples_{};
  size_t cursor_{0};
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  std::array<rclcpp::Publisher<Feedback>::SharedPtr, 6> feedback_pubs_;
  rclcpp::Publisher<Touch>::SharedPtr touch_pub_;
  rclcpp::Publisher<Result>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  std::array<rclcpp::Subscription<JointCommand>::SharedPtr, 5> joint_command_subs_;
  rclcpp::Service<SetValues>::SharedPtr set_srv_;
  rclcpp::Service<GetState>::SharedPtr get_srv_;
  rclcpp::Service<SetEnabled>::SharedPtr enable_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_, clear_stop_srv_, clear_srv_, reconnect_srv_;
  rclcpp::TimerBase::SharedPtr poll_timer_, state_timer_;
};
}  // namespace rh5dg2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<rh5dg2::HandNode>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rh5dg2_driver"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
