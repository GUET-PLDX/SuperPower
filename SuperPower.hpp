#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
template_args: []
required_hardware:
  - can
depends:
  - pldx/Referee
=== END MANIFEST === */
// clang-format on
#include <array>
#include <cmath>
#include <cstdint>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mpmc_queue.hpp"
#include "mutex.hpp"
#include "timer.hpp"

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

/**
 * @class SuperPower
 * @brief 主控侧超级电容通信模块
 * @details CAN 与裁判系统回调只缓存数据，5ms 定时器统一解析状态并下发控制帧。
 */
class SuperPower : public LibXR::Application {
 public:
  struct TelemetrySnapshot {
    float chassis_power_w = 0.0f;
    uint16_t cap_chassis_power_limit_w = 0U;
    uint16_t referee_power_limit_w = 0U;
    uint16_t referee_energy_buffer_j = 0U;
    float cap_energy_normalized = 0.0f;
    uint8_t cap_energy_raw = 0U;
    uint8_t error_code = 0U;
    uint32_t chassis_power_sequence = 0U;
    bool supercap_online = false;
    bool supercap_healthy = false;
    bool referee_power_limit_online = false;
    bool referee_energy_buffer_online = false;
    bool referee_online = false;
  };

  SuperPower(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* can_bus_name)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   SuperPowerProtocol::FEEDBACK_ID,
                   SuperPowerProtocol::FEEDBACK_ID);
    RegisterRefereeCallback();
    RegisterUseCapacitorCallback();

    timer_handle_ =
        LibXR::Timer::CreateTask(TimerTask, this, COMMAND_PERIOD_MS);
    LibXR::Timer::Add(timer_handle_);
    LibXR::Timer::Start(timer_handle_);
  }

  TelemetrySnapshot GetTelemetrySnapshot() {
    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    LibXR::Mutex::LockGuard lock(state_mutex_);
    RefreshOnlineStateLocked(NOW_MS);

    const bool CHASSIS_POWER_FINITE = std::isfinite(chassis_power_);
    TelemetrySnapshot snapshot{};
    snapshot.chassis_power_w =
        chassis_power_valid_ && CHASSIS_POWER_FINITE ? chassis_power_ : 0.0f;
    snapshot.cap_chassis_power_limit_w = chassis_power_limit_;
    snapshot.referee_power_limit_w = referee_power_limit_;
    snapshot.referee_energy_buffer_j = referee_energy_buffer_;
    snapshot.cap_energy_normalized = static_cast<float>(cap_energy_) / 255.0f;
    snapshot.cap_energy_raw = cap_energy_;
    snapshot.error_code = error_code_;
    snapshot.chassis_power_sequence = chassis_power_sequence_;
    snapshot.supercap_online = feedback_received_ && use_capacitor_;
    snapshot.supercap_healthy = snapshot.supercap_online &&
                                chassis_power_valid_ && CHASSIS_POWER_FINITE &&
                                error_code_ == 0U;
    snapshot.referee_power_limit_online = referee_power_limit_received_;
    snapshot.referee_energy_buffer_online = referee_energy_buffer_received_;
    snapshot.referee_online = snapshot.referee_power_limit_online &&
                              snapshot.referee_energy_buffer_online;
    return snapshot;
  }

  float GetChassisPower() { return GetTelemetrySnapshot().chassis_power_w; }

  float GetCapEnergy() { return GetTelemetrySnapshot().cap_energy_normalized; }

  uint8_t GetErrorCode() { return GetTelemetrySnapshot().error_code; }

  uint16_t GetChassisPowerLimit() {
    return GetTelemetrySnapshot().cap_chassis_power_limit_w;
  }

  bool IsOnline() { return GetTelemetrySnapshot().supercap_online; }

  void OnMonitor() override {}

 private:
  static constexpr uint32_t COMMAND_PERIOD_MS = 5U;
  static constexpr uint32_t STATUS_RX_TIMEOUT_MS = 100U;
  static constexpr uint32_t REFEREE_RX_TIMEOUT_MS = 1000U;
  static constexpr uint32_t TIMESTAMP_FUTURE_TOLERANCE_MS = 5U;

  struct FeedbackFrame {
    LibXR::CAN::ClassicPack pack;
    uint32_t received_time_ms;
  };

  struct RefereeData {
    uint16_t power_limit;
    uint16_t energy_buffer;
    uint32_t power_limit_received_time_ms;
    uint32_t energy_buffer_received_time_ms;
    bool power_limit_received;
    bool energy_buffer_received;
  };

  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    if (pack.dlc < sizeof(SuperPowerProtocol::FeedbackData)) {
      return;
    }

    FeedbackFrame frame{
        pack, static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds())};
    if (self->feedback_queue_.Push(frame) == LibXR::ErrorCode::FULL) {
      self->feedback_queue_.Pop();
      self->feedback_queue_.Push(frame);
    }
  }

  void RegisterRefereeCallback() {
    auto topic_handle = LibXR::Topic::Find("chassis_ref", nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self,
           const Referee::ChassisPack& chassis_pack) {
          UNUSED(in_isr);
          RefereeData data{chassis_pack.rs.chassis_power_limit,
                           chassis_pack.power_buffer,
                           chassis_pack.robot_status_received_time_ms,
                           chassis_pack.power_heat_received_time_ms,
                           chassis_pack.robot_status_received,
                           chassis_pack.power_heat_received};
          if (self->referee_queue_.Push(data) == LibXR::ErrorCode::FULL) {
            self->referee_queue_.Pop();
            self->referee_queue_.Push(data);
          }
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  void RegisterUseCapacitorCallback() {
    auto topic_handle =
        LibXR::Topic::FindOrCreate<bool>("use_capacitor", nullptr, true);
    ASSERT(topic_handle != nullptr);

    auto callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self, const bool& enabled) {
          UNUSED(in_isr);
          LibXR::Mutex::LockGuard lock(self->state_mutex_);
          self->use_capacitor_ = enabled;
        },
        this);

    LibXR::Topic(topic_handle).RegisterCallback(callback);
  }

  static void TimerTask(SuperPower* self) { self->Update(); }

  void Update() {
    FeedbackFrame feedback_frame{};
    RefereeData referee_data{};
    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());

    {
      LibXR::Mutex::LockGuard lock(state_mutex_);
      while (feedback_queue_.Pop(feedback_frame) == LibXR::ErrorCode::OK) {
        const auto feedback =
            SuperPowerProtocol::DecodeFeedback(feedback_frame.pack.data);
        ++chassis_power_sequence_;
        error_code_ = feedback.error_code;
        if (std::isfinite(feedback.chassis_power)) {
          chassis_power_ = feedback.chassis_power;
          chassis_power_valid_ = true;
        } else {
          chassis_power_valid_ = false;
        }
        chassis_power_limit_ = feedback.chassis_power_limit;
        cap_energy_ = feedback.cap_energy;
        last_feedback_rx_time_ms_ = feedback_frame.received_time_ms;
        feedback_received_ = true;
      }

      while (referee_queue_.Pop(referee_data) == LibXR::ErrorCode::OK) {
        if (referee_data.power_limit_received &&
            (!referee_power_limit_seen_ ||
             referee_data.power_limit_received_time_ms !=
                 last_referee_power_limit_rx_time_ms_)) {
          referee_power_limit_ = referee_data.power_limit;
          last_referee_power_limit_rx_time_ms_ =
              referee_data.power_limit_received_time_ms;
          referee_power_limit_seen_ = true;
          referee_power_limit_received_ = true;
        }
        if (referee_data.energy_buffer_received &&
            (!referee_energy_buffer_seen_ ||
             referee_data.energy_buffer_received_time_ms !=
                 last_referee_energy_buffer_rx_time_ms_)) {
          referee_energy_buffer_ = referee_data.energy_buffer;
          last_referee_energy_buffer_rx_time_ms_ =
              referee_data.energy_buffer_received_time_ms;
          referee_energy_buffer_seen_ = true;
          referee_energy_buffer_received_ = true;
        }
      }

      RefreshOnlineStateLocked(NOW_MS);
    }
    SendCommandFrame(NOW_MS);
  }

  static bool IsFreshAt(uint32_t now_ms, uint32_t received_time_ms,
                        uint32_t timeout_ms) {
    const uint32_t ELAPSED_MS = now_ms - received_time_ms;
    if (ELAPSED_MS <= timeout_ms) {
      return true;
    }
    const uint32_t FUTURE_OFFSET_MS = received_time_ms - now_ms;
    return FUTURE_OFFSET_MS <= TIMESTAMP_FUTURE_TOLERANCE_MS;
  }

  void RefreshOnlineStateLocked(uint32_t now_ms) {
    if (feedback_received_ &&
        !IsFreshAt(now_ms, last_feedback_rx_time_ms_, STATUS_RX_TIMEOUT_MS)) {
      ClearFeedbackStateLocked();
    }
    if (referee_power_limit_received_ &&
        !IsFreshAt(now_ms, last_referee_power_limit_rx_time_ms_,
                   REFEREE_RX_TIMEOUT_MS)) {
      referee_power_limit_received_ = false;
    }
    if (referee_energy_buffer_received_ &&
        !IsFreshAt(now_ms, last_referee_energy_buffer_rx_time_ms_,
                   REFEREE_RX_TIMEOUT_MS)) {
      referee_energy_buffer_received_ = false;
    }
  }

  void ClearFeedbackStateLocked() {
    error_code_ = 0U;
    chassis_power_ = 0.0f;
    chassis_power_limit_ = 0U;
    cap_energy_ = 0U;
    last_feedback_rx_time_ms_ = 0U;
    feedback_received_ = false;
    chassis_power_valid_ = false;
  }

  void SendCommandFrame(uint32_t now_ms) {
    SuperPowerProtocol::CommandData command{};
    {
      LibXR::Mutex::LockGuard lock(state_mutex_);
      command.flags =
          use_capacitor_ ? SuperPowerProtocol::ENABLE_DCDC_MASK : 0U;
      if (referee_power_limit_received_ && referee_energy_buffer_received_ &&
          IsFreshAt(now_ms, last_referee_power_limit_rx_time_ms_,
                    REFEREE_RX_TIMEOUT_MS) &&
          IsFreshAt(now_ms, last_referee_energy_buffer_rx_time_ms_,
                    REFEREE_RX_TIMEOUT_MS)) {
        command.referee_power_limit = referee_power_limit_;
        command.referee_energy_buffer = referee_energy_buffer_;
      }
    }

    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = SuperPowerProtocol::COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(command);
    LibXR::Memory::FastCopy(tx_pack.data, &command, sizeof(command));
    can_->AddMessage(tx_pack);
  }

  LibXR::CAN* can_;
  LibXR::Timer::TimerHandle timer_handle_ = nullptr;
  LibXR::MPMCQueue<FeedbackFrame> feedback_queue_{2};
  LibXR::MPMCQueue<RefereeData> referee_queue_{2};
  LibXR::Mutex state_mutex_;

  uint32_t last_feedback_rx_time_ms_ = 0U;
  uint32_t last_referee_power_limit_rx_time_ms_ = 0U;
  uint32_t last_referee_energy_buffer_rx_time_ms_ = 0U;
  uint32_t chassis_power_sequence_ = 0U;
  float chassis_power_ = 0.0f;
  uint16_t chassis_power_limit_ = 0U;
  uint16_t referee_power_limit_ = 0U;
  uint16_t referee_energy_buffer_ = 0U;
  uint8_t error_code_ = 0U;
  uint8_t cap_energy_ = 0U;
  bool feedback_received_ = false;
  bool referee_power_limit_seen_ = false;
  bool referee_energy_buffer_seen_ = false;
  bool referee_power_limit_received_ = false;
  bool referee_energy_buffer_received_ = false;
  bool chassis_power_valid_ = false;
  bool use_capacitor_ = true;
};
