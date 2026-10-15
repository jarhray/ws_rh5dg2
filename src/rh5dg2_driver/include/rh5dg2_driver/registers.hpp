#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace rh5dg2
{

  inline bool is_placeholder_feedback(const std::vector<int16_t> &values)
  {
    // Observed on the physical link on 2026-10-13: only the first word is
    // meaningful; 0xAAAA and zero padding fill the rest of a block response.
    return values.size() == 13 && values[1] == static_cast<int16_t>(-21846) &&
           std::all_of(values.begin() + 2, values.end(), [](int16_t value)
                       { return value == 0; });
  }

  inline bool is_placeholder_angle_feedback(const std::vector<int16_t> &values)
  {
    // Also captured on this link: the first angle is real and all remaining
    // words are zero. This cannot be a valid full hand pose. Apply only to angles;
    // zero-filled current/error feedback can be legitimate.
    return is_placeholder_feedback(values) || (values.size() == 13 &&
                                               std::all_of(values.begin() + 1, values.end(), [](int16_t value)
                                                           { return value == 0; }));
  }

  inline std::array<std::string, 13> joint_names(const std::string &side)
  {
    const std::array<std::string, 13> suffixes{
        "pinky_mcp_joint", "ring_mcp_joint", "middle_pitch_joint", "middle_mcp_joint",
        "index_pitch_joint", "index_mcp_joint", "pinky_pip_joint", "ring_pip_joint",
        "middle_pip_joint", "index_pip_joint", "thumb_yaw_joint", "thumb_mcp_joint",
        "thumb_dip_joint"};
    auto names = suffixes;
    for (auto &name : names)
      name = side + "_" + name;
    return names;
  }

  inline std::array<double, 13> reference_zero()
  {
    return {1800, 1800, 1800, 0, 1800, 0, 1850, 1850, 1850, 1850, 1700,
            1600, 2040};
  }

  using JointValues = std::array<int16_t, 13>;
  using JointMask = std::array<bool, 13>;

  struct PreparedCommand
  {
    uint16_t address{};
    JointValues values{};
    JointMask clamped{};
  };

  // RH5EG2 V1.0 manual, in raw 0.1-degree units. -1 is reserved for skip.
  inline constexpr JointValues kAngleMin{
      1000, 1000, 1000, -150, 1000, -150, 1150, 1150, 1150, 1150, 650, 1300, 1500};
  inline constexpr JointValues kAngleMax{
      1650, 1650, 1650, 150, 1650, 150, 1850, 1850, 1850, 1850, 1700, 1600, 2040};

  // Prepare the complete command before any write. Only angles are clamped.
  inline PreparedCommand prepare_command(uint8_t kind, const JointValues &values)
  {
    static constexpr std::array<uint16_t, 5> addresses{1080, 1108, 1094, 1024, 1220};
    static constexpr std::array<int, 5> maximum{0, 2500, 1000, 1500, 1};
    if (kind >= addresses.size())
      throw std::invalid_argument("Unknown command kind");
    PreparedCommand command{addresses[kind], values, {}};
    for (size_t i = 0; i < values.size(); ++i)
    {
      const int value = values[i];
      if (kind == 0)
      {
        if (value == -1)
          continue;
        command.values[i] = std::clamp(values[i], kAngleMin[i], kAngleMax[i]);
        command.clamped[i] = command.values[i] != value;
      }
      else if (value < 0 || value > maximum[kind])
      {
        throw std::invalid_argument("Channel " + std::to_string(i) + " value " + std::to_string(value) +
                                    " at register " + std::to_string(addresses[kind] + i) + " outside [0, " +
                                    std::to_string(maximum[kind]) + "]");
      }
    }
    if (kind == 4 && (values[2] != values[3] || values[4] != values[5]))
      throw std::invalid_argument("Middle/index root and splay control modes must match");
    return command;
  }

} // namespace rh5dg2
