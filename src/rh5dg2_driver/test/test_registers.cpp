#include "rh5dg2_driver/registers.hpp"
#include <iostream>
#include <functional>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()> &function) {
  bool rejected = false;
  try { function(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Expected command rejection");
}

void check_angle_limits() {
  const std::vector<int64_t> lower(rh5dg2::kAngleMin.begin(), rh5dg2::kAngleMin.end());
  const std::vector<int64_t> upper(rh5dg2::kAngleMax.begin(), rh5dg2::kAngleMax.end());
  for (size_t i = 0; i < 13; ++i) {
    for (const auto value : {lower[i], upper[i], lower[i] - 1, upper[i] + 1,
                             int64_t{-32768}, int64_t{32767}}) {
      rh5dg2::JointValues input;
      input.fill(-1);
      input[i] = static_cast<int16_t>(value);
      const auto result = rh5dg2::prepare_command(0, input);
      require(result.address == 1080, "Wrong prepared angle register");
      require(result.values[i] == std::clamp(value, lower[i], upper[i]), "Incorrect angle saturation");
      require(result.clamped[i] == (value < lower[i] || value > upper[i]), "Incorrect clamp mask");
      for (size_t j = 0; j < 13; ++j) {
        if (j != i) require(result.values[j] == -1 && !result.clamped[j], "Skipped target changed");
      }
    }
  }
  rh5dg2::JointValues input;
  input.fill(-1);
  auto result = rh5dg2::prepare_command(0, input);
  require(result.values == input, "All-skipped command changed");
  require(std::none_of(result.clamped.begin(), result.clamped.end(), [](bool v) { return v; }),
          "Skip sentinel was clamped");
  input[3] = -2;
  require(rh5dg2::prepare_command(0, input).values[3] == -2, "Valid negative splay changed");
  input.fill(32767);
  result = rh5dg2::prepare_command(0, input);
  require(std::equal(result.values.begin(), result.values.end(), upper.begin()), "Full upper limits differ");
  input.fill(-32768);
  result = rh5dg2::prepare_command(0, input);
  require(std::equal(result.values.begin(), result.values.end(), lower.begin()), "Full lower limits differ");

  rejects([&] { rh5dg2::prepare_command(5, input); });
}

void check_other_commands() {
  const std::array<uint16_t, 4> addresses{1108, 1094, 1024, 1220};
  const std::array<int16_t, 4> maximum{2500, 1000, 1500, 1};
  rh5dg2::JointValues values;
  for (uint8_t kind = 1; kind <= 4; ++kind) {
    for (const int16_t bound : {int16_t{0}, maximum[kind - 1]}) {
      values.fill(bound);
      const auto result = rh5dg2::prepare_command(kind, values);
      require(result.address == addresses[kind - 1], "Wrong command address");
      require(result.values == values, "Valid non-angle command was changed");
      require(std::none_of(result.clamped.begin(), result.clamped.end(), [](bool v) { return v; }),
              "Non-angle command was clamped");
    }
    for (size_t i = 0; i < values.size(); ++i) {
      values.fill(0);
      values[i] = -1;
      rejects([&] { rh5dg2::prepare_command(kind, values); });
      values[i] = maximum[kind - 1] + 1;
      rejects([&] { rh5dg2::prepare_command(kind, values); });
    }
  }
  for (size_t root : {size_t{2}, size_t{4}}) {
    values.fill(0);
    values[root] = 1;
    rejects([&] { rh5dg2::prepare_command(4, values); });
    values[root + 1] = 1;
    require(rh5dg2::prepare_command(4, values).values == values, "Matching mode pair rejected");
  }
  values.fill(0);
  values[6] = 2501;
  std::string detail;
  try { rh5dg2::prepare_command(1, values); }
  catch (const std::invalid_argument &error) { detail = error.what(); }
  require(detail.find("Channel 6 value 2501 at register 1114 outside [0, 2500]") != std::string::npos,
          "Invalid command must identify channel, value, address and range");

}

int main() {
  try {
    check_angle_limits();
    check_other_commands();
    const std::vector<int16_t> captured_block{1648,-21846,0,0,0,0,0,0,0,0,0,0,0};
    require(rh5dg2::is_placeholder_feedback(captured_block), "Checksummed hardware filler not rejected");
    const std::vector<int16_t> captured_single{1648,1650,1652,0,1651,0,1857,1853,1850,1852,1702,1582,2046};
    require(!rh5dg2::is_placeholder_feedback(captured_single), "Real single-register readings rejected");
    require(!rh5dg2::is_placeholder_feedback(std::vector<int16_t>(13,0)), "Zero error sample rejected");
    require(rh5dg2::is_placeholder_angle_feedback({1648,0,0,0,0,0,0,0,0,0,0,0,0}),
            "Captured zero-padded block angles accepted");
    require(rh5dg2::is_placeholder_angle_feedback(std::vector<int16_t>(13,0)), "Uninitialized angles accepted");
    require(!rh5dg2::is_placeholder_angle_feedback(captured_single), "Individual angle samples rejected");
    const auto left = rh5dg2::joint_names("left"), right = rh5dg2::joint_names("right");
    require(left[12]=="left_thumb_dip_joint", "Independent thumb joint mismatch");
    for (size_t i=0;i<13;++i) require(left[i]!=right[i], "Hand names overlap");
    require(rh5dg2::reference_zero()[11] == 1600, "Thumb reference zero differs from the manual");
    std::cout << "Register and angle-limit checks passed\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
