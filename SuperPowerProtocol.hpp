#pragma once

#include <array>
#include <cstdint>

#include "libxr_mem.hpp"

namespace SuperPowerProtocol {

constexpr uint32_t FEEDBACK_ID = 0x051U;
constexpr uint32_t COMMAND_ID = 0x061U;
constexpr uint8_t ENABLE_DCDC_MASK = 0x01U;
constexpr uint8_t SYSTEM_RESTART_MASK = 0x02U;

struct __attribute__((packed)) FeedbackData {
  uint8_t error_code;
  float chassis_power;
  uint16_t chassis_power_limit;
  uint8_t cap_energy;
};

struct __attribute__((packed)) CommandData {
  uint8_t flags;
  uint16_t referee_power_limit;
  uint16_t referee_energy_buffer;
  uint8_t reserved[3];
};

static_assert(sizeof(FeedbackData) == 8U, "FeedbackData must be eight bytes");
static_assert(sizeof(CommandData) == 8U, "CommandData must be eight bytes");

inline FeedbackData DecodeFeedback(const uint8_t* data) {
  FeedbackData feedback{};
  LibXR::Memory::FastCopy(&feedback, data, sizeof(feedback));
  return feedback;
}

inline std::array<uint8_t, sizeof(CommandData)> EncodeCommand(
    const CommandData& command) {
  std::array<uint8_t, sizeof(CommandData)> bytes{};
  constexpr size_t COMMAND_PAYLOAD_SIZE =
      sizeof(CommandData) - sizeof(command.reserved);
  LibXR::Memory::FastCopy(bytes.data(), &command, COMMAND_PAYLOAD_SIZE);
  return bytes;
}

}  // namespace SuperPowerProtocol
