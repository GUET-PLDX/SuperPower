# SuperPower

`SuperPower` 是主控侧的超级电容 CAN 通信模块，负责接收超电状态帧、同步裁判系统功率上限，并把控制帧发回超电控制板。

这个模块只做通信和状态缓存，不负责电机限幅计算，也不负责 UI 绘制。功率控制由 `PowerControl` 使用这里提供的实测功率和在线状态完成，底盘 UI 只读取这里的只读状态。

## 职责

- 接收超电状态帧 `0x051`
- 解析超电错误码、底盘实际功率、底盘功率限制和储能值
- 订阅 `chassis_ref` 话题，缓存裁判系统的底盘功率限制和缓冲能量
- 每 `5 ms` 发送一次控制帧 `0x061`
- 使用状态帧和裁判数据接收超时分别判断两路数据源是否在线
- 向功率控制提供一次加锁生成的完整遥测快照

## 协议

| 项目 | 约定 |
|---|---|
| CAN 类型 | Classic CAN |
| 帧格式 | 标准帧 |
| 数据长度 | 8 字节 |
| 字节序 | STM32 本地小端 |
| 状态帧 ID | `0x051`，超电到主控 |
| 控制帧 ID | `0x061`，主控到超电 |

代码使用 packed 结构体描述 8 字节数据区，并通过 `memcpy` 在 CAN 数据区和结构体之间转换。

## 反馈帧

反馈帧由超电控制板发送给主控，标准帧 ID 为 `0x051`。

```cpp
struct __attribute__((packed)) FeedbackData {
  uint8_t error_code;
  float chassis_power;
  uint16_t chassis_power_limit;
  uint8_t cap_energy;
};
```

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `error_code` | `uint8_t` | 超电控制板错误码 |
| 1 | `chassis_power` | `float` | 底盘实际功率，单位 W |
| 5 | `chassis_power_limit` | `uint16_t` | 当前底盘功率限制 |
| 7 | `cap_energy` | `uint8_t` | 储能原始值，范围 `0~255` |

RM2024 将 `chassis_power` 直接以 IEEE-754 单精度浮点数传输，不使用偏移二进制编码。代码使用 packed 结构体和 `memcpy` 在 CAN 数据区与结构体之间转换。模块离线时，反馈接口返回 `0`。

收到非有限值（`NaN` 或 `Inf`）的 `chassis_power` 时，模块仍记录该帧的传输新鲜度、功率样本序号和错误码，但不会覆盖最后一次可信的有限功率缓存。遥测快照对该次无效测量返回 `0 W`，并将 `supercap_healthy` 置为 `false`；下一帧有限功率可恢复健康状态。`chassis_power_sequence` 对每个完成解码的状态帧递增，超时不会清零，供上层拒绝重复消费同一功率样本。

## 控制帧

控制帧由主控发送给超电控制板，标准帧 ID 为 `0x061`。

```cpp
struct __attribute__((packed)) CommandData {
  uint8_t flags;
  uint16_t referee_power_limit;
  uint16_t referee_energy_buffer;
  uint8_t reserved[3];
};
```

| 偏移 | 字段 | 类型 | 当前写入 |
|---:|---|---|---|
| 0 | `flags` | `uint8_t` | `bit0` 固定置 `1`，使能 `enableCONV` |
| 1 | `referee_power_limit` | `uint16_t` | `chassis_ref.rs.chassis_power_limit` |
| 3 | `referee_energy_buffer` | `uint16_t` | `chassis_ref.power_buffer` |
| 5 | `reserved` | `uint8_t[3]` | 全部为 `0` |

控制帧由 `LibXR::Timer` 每 `5 ms` 发送一次。`referee_power_limit` 和 `referee_energy_buffer` 都取自最新的 `chassis_ref` 数据；当裁判数据超过 `1000 ms` 未更新时，两者都保持为 `0`，避免继续使用旧值。

## 在线判定

模块启动后，在收到第一帧有效反馈帧之前认为超电离线。每一帧有效反馈都会刷新最后接收时间；超过 `100 ms` 没有收到反馈时，模块进入离线状态并清空对外反馈数据。内容完全相同的反馈帧仍然表示在线，不能作为离线判据。

裁判功率上限来自 `0x0201`，缓冲能量来自 `0x0202`，两者分别使用各自的有效标志、接收时间和 `1000 ms` 新鲜度判定。只有两项都新鲜时 `referee_online` 才为 `true`；任一项超时后，控制帧继续发送零功率上限和零缓冲能量，但两项最后一次可信值仍保留在遥测快照中，供上层进行保守降级。其他裁判数据包触发的 `chassis_ref` 发布不会刷新这两个时间。

## 运行流程

1. 构造时根据 `can_bus_name` 查找 CAN 总线
2. 注册标准帧过滤器，只接收 ID `0x051`
3. 订阅 `chassis_ref`，保存裁判系统底盘功率上限和最后接收时间
4. CAN 接收回调检查反馈帧长度，长度不足 8 字节时丢弃
5. 有效反馈帧进入队列，由定时任务解析、缓存并刷新在线状态
6. 定时任务每 `5 ms` 发送一帧控制帧

## 对外接口

`GetTelemetrySnapshot()` 一次读取时钟后只获取一次状态互斥锁，在锁内同时更新超电和裁判数据的新鲜度并复制全部字段，因此上层可从同一个时间点判断三类数据源状态。`referee_power_limit_online` 和 `referee_energy_buffer_online` 分别表示 `0x0201` 与 `0x0202` 新鲜；兼容字段 `referee_online` 仅在两者都新鲜时为 `true`。

新鲜度计算兼容 32 位毫秒计数器回绕，并允许回调在快照读取时钟后、获取互斥锁前写入最多 `5 ms` 的未来接收时间，避免把并发到达的新数据误判为超时。超过这个容差的未来时间和超过半个计数周期的古老时间都视为无效。裁判系统重复发布相同的源时间戳不会重新激活已经超时的数据；只有对应源的新时间戳才能恢复在线状态。

| `TelemetrySnapshot` 字段 | 单位或含义 |
|---|---|
| `chassis_power_w` | 有效的底盘实测功率，W；无效或超电离线时为 `0` |
| `cap_chassis_power_limit_w` | 超电反馈的底盘功率限制，W |
| `cap_energy_raw` | 超电储能原始值，`0~255` |
| `cap_energy_normalized` | 超电储能归一化值，`0.0f~1.0f` |
| `error_code` | RM2024 超电错误码 |
| `chassis_power_sequence` | 完成解码的超电功率样本序号，用于上层去重 |
| `referee_power_limit_w` | 最近一次可信的裁判底盘功率上限，W |
| `referee_energy_buffer_j` | 最近一次可信的裁判缓冲能量，J |
| `supercap_online` | 超电反馈在 `100 ms` 内有更新 |
| `supercap_healthy` | 超电在线、实测功率有限且错误码为 `0` |
| `referee_power_limit_online` | 裁判 `0x0201` 在 `1000 ms` 内有更新 |
| `referee_energy_buffer_online` | 裁判 `0x0202` 在 `1000 ms` 内有更新 |
| `referee_online` | 裁判 `0x0201` 和 `0x0202` 数据都在 `1000 ms` 内有更新 |

| 接口 | 在线返回 | 离线返回 |
|---|---|---|
| `GetChassisPower()` | 反馈帧中的底盘实际功率，单位 W | `0` |
| `GetChassisPowerLimit()` | 反馈帧中的底盘功率限制 | `0` |
| `GetErrorCode()` | 反馈帧中的错误码 | `0` |
| `GetCapEnergy()` | `cap_energy / 255.0f` | `0` |
| `IsOnline()` | `true` | `false` |

`GetCapEnergy()` 返回反馈帧中存储的能量值归一化到 `0.0f~1.0f`，即 `cap_energy / 255.0f`。该接口名称为上层兼容保留。

## YAML 配置

最小配置如下：

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
    can_bus_name: can1
```

`can_bus_name` 必须对应 `User/app_main.cpp` 中已经注册的 CAN 设备。系统里还需要有 `Referee` 模块持续发布 `chassis_ref` 话题，否则控制帧里的 `referee_power_limit` 会保持默认值 `0`；如果裁判数据中途超时，也会下发 `0`，避免继续使用旧功率上限。

## 模块声明

Required Hardware:

- can

Depends:

  - pldx/Referee

代码入口：

- `Modules/SuperPower/SuperPower.hpp`
