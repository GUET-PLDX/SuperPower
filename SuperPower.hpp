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
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on
#include <cstring>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "message.hpp"

/* 超电状态帧 CAN 标准 ID 超电发给主控 */
#define FEEDBACK_ID 0x51
/* 超电控制帧 CAN 标准 ID 主控发给超电 */
#define COMMAND_ID 0x61

/**
 * @class SuperPower
 * @brief 主控侧超级电容通信模块
 * @details 接收超电状态帧 同步裁判系统功率上限 回调里下发控制帧
 *          给上层功率控制提供在线状态和功率数据
 */
class SuperPower : public LibXR::Application {
 public:
  /**
   * @brief 超电状态帧
   * @details 对应标准帧 ID 0x51 数据长度固定为 8 字节
   */
  struct __attribute__((packed)) StatusData {
    uint8_t power_limit;            /* 超电侧当前功率限制 */
    uint16_t chassis_power;         /* 底盘实际功率编码值 */
    uint16_t referee_power;         /* 裁判系统总输出功率编码值 */
    uint16_t superpower_output_max; /* 超电可向 A 侧输出的最大功率 */
    uint8_t output_capability;      /* 输出能力原始值，范围 0~255 */
  };

  /**
   * @brief 超电控制帧
   * @details 对应标准帧 ID 0x61 数据长度固定为 8 字节
   */
  struct __attribute__((packed)) CommandData {
    uint8_t flags;                /* bit0 为 enableCONV 使能位 */
    uint16_t referee_power_limit; /* 下发给超电的裁判功率上限 */
    uint16_t reserved0;           /* 协议保留字段，发送 0 */
    uint8_t reserved1;            /* 协议保留字段，发送 0 */
    int16_t reserved2;            /* 协议保留字段，发送 0 */
  };

  /**
   * @brief SuperPower 构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param can_bus_name CAN 总线名称
   * @details 构造时注册 0x51 状态帧接收过滤器 订阅 chassis_ref 话题
   */
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
                   LibXR::CAN::FilterMode::ID_RANGE, FEEDBACK_ID, FEEDBACK_ID);

    RegisterRefereeCallback();
  }

  /**
   * @brief 订阅裁判系统底盘数据
   * @details 从 chassis_ref 话题获取裁判系统底盘功率上限
   *          后面发控制帧时写到 referee_power_limit 字段里
   */
  void RegisterRefereeCallback() {
    auto topic_handle = LibXR::Topic::Find("chassis_ref", nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self,
           const Referee::ChassisPack& chassis_pack) {
          UNUSED(in_isr);
          self->referee_power_limit_ = chassis_pack.rs.chassis_power_limit;
          self->last_referee_rx_time_ms_ =
              static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  /**
   * @brief 处理超电状态帧
   * @param pack 接收到的 CAN 标准帧
   * @details 只处理长度不小于 StatusData 的状态帧
   *          同时记录接收时间，连续相同帧达到阈值后也判为离线
   */
  void OnFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if (pack.dlc < sizeof(StatusData)) {
      return;
    }

    StatusData data{};
    LibXR::Memory::FastCopy(&data, pack.data, sizeof(StatusData));
    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    last_status_rx_time_ms_ = NOW_MS;
    UpdateSameFrameCount(data);
    DecodeStatusData(data);
    status_received_ = true;

    if (RefreshOnlineState()) {
      SendCommandFrame(NOW_MS);
    }
  }

  /**
   * @brief 解析超电状态帧
   * @param data 已经拷贝出来的协议数据
   * @details 功率字段按各自协议语义处理后再缓存
   */
  void DecodeStatusData(const StatusData& data) {
    power_limit_ = data.power_limit;
    chassis_power_ = DecodeOffsetPower(data.chassis_power);
    referee_power_ = DecodeOffsetPower(data.referee_power);
    superpower_output_max_ = DecodeDirectPower(data.superpower_output_max);
    output_capability_ = data.output_capability;
  }

  /**
   * @brief 获取底盘实际功率
   * @return 解码后的底盘实际功率，单位 W，离线时返回 0
   */
  float GetChassisPower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return chassis_power_;
  }

  /**
   * @brief 获取归一化后的超电输出能力
   * @details 这个接口为了兼容旧上层命名保留
   *          实际含义是输出能力比例 不是电容容量或剩余电量
   * @return output_capability / 255.0f 离线时返回 0
   */
  float GetCapEnergy() {
    RefreshOnlineState();
    return static_cast<float>(output_capability_) / 255.0f;
  }

  /**
   * @brief 获取裁判系统总输出功率
   * @return 解码后的裁判系统总输出功率，单位 W，离线时返回 0
   */
  float GetRefereePower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return referee_power_;
  }

  /**
   * @brief 获取超电可向 A 侧输出的最大功率
   * @return 当前最大输出功率，单位 W，离线时返回 0
   */
  float GetSuperPowerOutputMax() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return superpower_output_max_;
  }

  /**
   * @brief 获取超电认为的当前功率限制
   * @return 功率限制原始值，离线时返回 0
   */
  uint8_t GetPowerLimit() {
    RefreshOnlineState();
    return power_limit_;
  }

  /**
   * @brief 获取超电在线状态
   * @return true 表示在线 false 表示离线或还没收到状态帧
   */
  bool IsOnline() { return RefreshOnlineState(); }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {}

 private:
  /* 控制帧 flags 的 bit0 对应协议里的 enableCONV 使能位 */
  static constexpr uint8_t ENABLE_CONV_MASK = 0x01;
  /* 超电控制帧最小发送间隔 */
  static constexpr uint32_t COMMAND_PERIOD_MS = 5;
  /* 超电状态帧接收超时时间 */
  static constexpr uint32_t STATUS_RX_TIMEOUT_MS = 100;
  /* 裁判系统底盘数据接收超时时间 */
  static constexpr uint32_t REFEREE_RX_TIMEOUT_MS = 1000;
  /* 裁判数据失效时下发 0，避免继续使用过期功率上限 */
  static constexpr uint16_t STALE_REFEREE_POWER_LIMIT = 0;
  /* 连续相同状态帧达到这个数量后认为超电离线 */
  static constexpr uint16_t SAME_FRAME_OFFLINE_COUNT = 200;
  /* 功率字段零点偏移 */
  static constexpr float POWER_ENCODE_OFFSET = 16384.0f;
  /* 功率字段缩放倍数 */
  static constexpr float POWER_ENCODE_SCALE = 64.0f;

  /**
   * @brief 解码带零点偏移的功率字段
   * @param encoded 协议里的功率编码值
   * @return 解码后的功率，单位 W
   */
  static float DecodeOffsetPower(uint16_t encoded) {
    return (static_cast<float>(encoded) - POWER_ENCODE_OFFSET) /
           POWER_ENCODE_SCALE;
  }

  /**
   * @brief 解码直接功率字段
   * @param power 协议里的功率值
   * @return 功率值，单位 W
   */
  static float DecodeDirectPower(uint16_t power) {
    return static_cast<float>(power);
  }

  /**
   * @brief 更新连续相同状态帧计数
   * @param data 当前状态帧数据
   */
  void UpdateSameFrameCount(const StatusData& data) {
    if (!status_received_ ||
        std::memcmp(&data, &last_status_data_, sizeof(StatusData)) != 0) {
      last_status_data_ = data;
      same_frame_count_ = 1;
      return;
    }

    if (same_frame_count_ < SAME_FRAME_OFFLINE_COUNT) {
      ++same_frame_count_;
    }
  }

  /**
   * @brief CAN 接收回调
   * @details 回调里完成收包解析和控制帧下发
   */
  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    self->OnFeedbackFrame(pack);
  }

  /**
   * @brief 离线后清空对外状态
   * @details 只清空状态帧更新出来的数据，裁判功率上限由发送时 freshness 判定
   */
  void ClearStatus() {
    power_limit_ = 0;
    chassis_power_ = 0.0f;
    referee_power_ = 0.0f;
    superpower_output_max_ = 0.0f;
    output_capability_ = 0;
  }

  /**
   * @brief 按状态帧接收 freshness 和连续相同帧数量刷新在线状态
   * @return true 表示在线 false 表示未收到、接收超时或连续相同帧过多
   */
  bool RefreshOnlineState() {
    if (!status_received_) {
      ClearStatus();
      return false;
    }

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if ((NOW_MS - last_status_rx_time_ms_) > STATUS_RX_TIMEOUT_MS) {
      ClearStatus();
      status_received_ = false;
      same_frame_count_ = 0;
      return false;
    }

    if (same_frame_count_ >= SAME_FRAME_OFFLINE_COUNT) {
      ClearStatus();
      return false;
    }

    return true;
  }

  /**
   * @brief 按 5ms 节流下发控制帧
   * @param now_ms 当前毫秒时间戳
   */
  void SendCommandFrame(uint32_t now_ms) {
    const uint32_t LAST_TX_MS = last_command_tx_time_ms_;
    const auto NOW_TIMESTAMP = LibXR::MillisecondTimestamp(now_ms);
    const auto LAST_TX_TIMESTAMP = LibXR::MillisecondTimestamp(LAST_TX_MS);

    if ((NOW_TIMESTAMP - LAST_TX_TIMESTAMP).ToMillisecond() <
        COMMAND_PERIOD_MS) {
      return;
    }

    SendCommandFrameNow(now_ms);
    last_command_tx_time_ms_ = now_ms;
  }

  /**
   * @brief 发送超电控制帧
   * @details 当前固定打开 enableCONV
   *          裁判系统数据超时时下发 0，避免使用过期功率上限
   */
  void SendCommandFrameNow(uint32_t now_ms) {
    CommandData command_data{};
    command_data.flags = ENABLE_CONV_MASK;
    command_data.referee_power_limit = GetRefereePowerLimitForCommand(now_ms);

    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CommandData);
    static_assert(sizeof(CommandData) == 8,
                  "CommandData must be 8 bytes for CAN");
    LibXR::Memory::FastCopy(tx_pack.data, &command_data, sizeof(CommandData));
    can_->AddMessage(tx_pack);
  }

  /**
   * @brief 获取控制帧使用的裁判功率上限
   * @details 未收到裁判数据或数据超时时使用保守值 0
   */
  uint16_t GetRefereePowerLimitForCommand(uint32_t now_ms) const {
    if (last_referee_rx_time_ms_ == 0 ||
        (now_ms - last_referee_rx_time_ms_) > REFEREE_RX_TIMEOUT_MS) {
      return STALE_REFEREE_POWER_LIMIT;
    }

    return referee_power_limit_;
  }

  LibXR::CAN* can_;

  uint16_t referee_power_limit_ = 0;       /* 裁判系统功率上限 */
  StatusData last_status_data_{};          /* 上一帧状态数据 */
  float chassis_power_ = 0.0f;             /* 解码后的底盘实际功率，单位 W */
  float referee_power_ = 0.0f;             /* 解码后的裁判系统总功率，单位 W */
  float superpower_output_max_ = 0.0f;     /* 最大输出功率，单位 W */
  uint32_t last_command_tx_time_ms_ = 0;   /* 最后发送时间 */
  uint32_t last_status_rx_time_ms_ = 0;    /* 最后收到超电状态帧时间 */
  uint32_t last_referee_rx_time_ms_ = 0;   /* 最后收到裁判底盘数据时间 */
  uint16_t same_frame_count_ = 0;          /* 连续相同状态帧数量 */
  uint8_t power_limit_ = 0;                /* 功率限制原始值 */
  uint8_t output_capability_ = 0;          /* 输出能力原始值 */
  bool status_received_ = false;           /* 是否收到过状态帧 */
};



