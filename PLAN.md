# ME3 无人机上位机运动路径控制 — 实施计划

> 项目代号: ME3-deploy  
> 目标: 上位机通过 WiFi 控制 DJI Mavic 3E 无人机自动飞行（室内），由智能车状态机联动控制  
> 约束: 机载侧仅最小化修改（PSDK ~10 文件 ≈ 50 行），上位机自由开发  
> 日期: 2026-07-04 / 更新: 2026-07-05

---

## 一、完成度总览

### ✅ 已完成

| 类别 | 内容 | 验证方式 |
|------|------|---------|
| **位置数据源** | PSDK Topic 41 (POSITION_VO) → TTALINK → ROS /uavdata | 室内静止漂移 <1cm/70s，室外飞行 health=3 全程 |
| **数据通路** | PSDK 6 文件 + ROS 3 文件修改，编译通过 | 飞行中持续输出 vo_x/y/z/health |
| **BODY/NED 切换** | flightByVel 新增 frame 字段，fcProxy + F_velCtrlFlight 双模式 | 编译通过 |
| **frame=2 位置偏移** | fcProxy.cpp 支持 frame=2 切换到 ROS_F_GPS_POS_VEL | 编译通过 |
| **VO 洗毒融合** | vo_fusion.py：四元数旋转 + 陀螺航向积分 + AprilTag 校正 | 理论验证，待实飞 |
| **上位机库** | telemetry / flight_control / path_executor / safety / utils | 已就绪 |
| **WiFi 通信** | 172.20.10.6，SSH + ROS 跨机通信 | ✅ 已验证 |
| **飞行测试** | 17 次起飞测试 | 见 §四 |

### ⏳ 未完成

| 内容 | 优先级 | 说明 |
|------|--------|------|
| **起飞方案修复** | 🔴 阻塞 | MonitoredTakeoff → TurnOnMotors（见 §五） |
| **四元数问题调查** | 🟡 中 | 回调输出全 0，计划改用 NED 直通绕过 |
| **车-机状态机联动** | 🔴 核心 | 已编写 `state_machine.py`（~390行），待起飞修复后联调 |
| **AprilTag 精准降落** | 🟢 后 | 已有 tta_apriltag_detect.py，需集成到上位机 |

---

## 二、位置数据源 — Topic 41 (POSITION_VO)

### 2.1 由来

M3E 飞控内部已完成 **全向视觉 + 超声波 + IMU + 气压计 + 红外 TOF** 多传感器融合，输出为 PSDK Topic 41 (`DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO`)，定义在 `dji_fc_subscription.h:487`。

### 2.2 结构与性能

| 属性 | 值 |
|------|-----|
| 传感器源 | 全向视觉(VO) + 超声波 + IMU + 气压计 + 红外 TOF + GPS/RTK(如有) |
| 数据结构 | `{float x, y, z; uint8 xHealth:1, yHealth:1, zHealth:1;}` |
| 更新率 | 最高 50 Hz（当前配置） |
| 坐标系 | NED (X=北, Y=东, Z=地) |
| 单位 | 米 |
| 室内实测噪声 | x: 0.8cm, y: 0.9cm (静止) |
| 室内实测漂移 | < 1cm / 70s (静止) |
| 室内健康率 | 100% (纹理充足的室内) |

### 2.3 数据流

```
M3E 飞控传感器融合 (内部 EKF)
  ↓ Topic 41, 50Hz
sensor_t.positionVO            ← 新增字段
  ↓
gcs_transmit.cpp → param[0..3] ← 打包进 TTALINK 8201
  ↓ TCP 127.0.0.1:10086
fcProxy → uavData.cpp          ← 提取
  ↓
publish.cpp → /uavdata topic   ← 新增 vo_x,vo_y,vo_z,vo_health
  ↓
上位机 vo_fusion.py            ← 订阅 + 洗毒融合
```

### 2.4 室内洗毒策略

Topic 41 的 X-Y 方向标注由 VO + 磁罗盘共同确定 (`dji_fc_subscription.h:463-467`)——室内罗盘不准，方向标注"有毒"。**取其增量、弃其方向**：

```
Δpos_ned = frame_cur - frame_prev                     ← 纯视觉增量
Δpos_body = quat_conj ⊗ Δpos_ned ⊗ quat              ← 四元数转体坐标
Δθ_head = Σ (ω_z - bias) · Δt                         ← 陀螺航向积分
Δpos_map = R(Δθ_head) · Δpos_body                     ← 转到地图坐标
pos_map += Δpos_map                                    ← 累积
if AprilTag: pos_map = tag_pose                        ← 漂移清零
```

VO 的位移增量来源于视觉特征匹配，不依赖罗盘；只有 X-Y 该叫"北"还是"东"这个标注层是罗盘给的、在室内不可靠。用四元数（pitch/roll 由重力确定）把增量从 NED 旋转到体坐标系即完成洗毒。

---

## 三、飞行测试记录

### 3.1 完整测试历史（2026-07-05）

| 测试 | 起飞 | 高度 | 水平 | PSDK 日志 | 根因 |
|------|:--:|------|:--:|------|------|
| v2 | ✅ | 109cm | — | 正常 | ref 读在起飞后 |
| v3~v4 | ✅ | 110cm | ❌ | joystick auth error 0x06 | RC 不在 N 档 |
| v5 | ✅ | 110cm | ❌ | 同 v3 | frame=2 位置偏移无效果 |
| v6~v8 | ❌ | 0cm | — | `arrest flying failed` 0x16200301 | 电池低压保护/连续起降禁飞 |
| v9 | ✅ | 112cm | ❌ | `Obtain joystick failed 0x06` | RC 不在 N 档，代码补丁未生效 |
| v10 | ✅ | 110cm | ❌ | 四元数全 0：`q[0]:0.000, q[1]:0.000...` | BODY→NED 旋转矩阵为零 |
| v11 | ✅ | 110cm | ❌ | `Not allowed to obtain joystick in P_MODE` | RC N 档时降下仍被拒 |
| v12~v13 | ❌ | 0cm | — | 同上 arrest | 飞控 arrested |
| v14 | ✅ | 106cm | ❌ | `Obtain joystick… failed` | MonitoredTakeoff z-lock |
| v15 | ✅ | 106cm | ❌ | BODY mode 无效 | 四元数零 → 体坐标系速度被吃掉 |
| v16 | ✅ | 106cm | ❌ | NED mode 无效 | MonitoredTakeoff 锁所有通道 |

### 3.2 两个已锁定的致命问题

#### 问题 A：MonitoredTakeoff 锁死全部 joystick 通道

`Dji_FlightControlMonitoredTakeoff()` 完成后，飞控内部切换到 POSITION 保持模式。**由此产生的 joystick 锁死影响所有轴（x, y, z, yaw），无论后续 SetJoystickMode 和 ObtainJoystickCtrlAuthority 调用多少次都无法解除。** 只有 `srv(2)` landing 能退出。

PSDK 日志证据：MonitoredTakeoff 执行完毕后连续打印 `Obtain joystick authority failed, error code: 0x00000006`，直到 landing 命令发出。

**修复方案**：在 `F_AttiCtrlTakeOff()` 中将 `Dji_FlightControlMonitoredTakeoff()` 替换为 `DjiFlightController_TurnOnMotors()` + 手动设置 VELOCITY joystick 模式 + `ObtainJoystickCtrlAuthority()`。电机的全部速度由 ROS 侧通过 `/flightByVel` 发出。改动量：约 15 行，限于 `flight_logic.c` 一个函数。

#### 问题 B：四元数回调输出全为零

PSDK 日志显示 `Dji_FcSubscriptionReceiveQuaternionCallback` 每次收到的四元数都是 `q = [0.0000, 0.0000, 0.0000, 0.0000]`（PSDK log: `vel flight control --> q[0]:0.0000...`）。四元数全为零时，NED→BODY 旋转矩阵退化为零矩阵，任何体坐标系速度指令在 `vector_XYZ2NED_Quaternion()` 后都变为零向量。

**根因推断**：`QUATERNION` topic 订阅是 5Hz，但 `F_velCtrlFlight()` 中直接从 `g_sensor_data.quaternion` 读取——此时如果 quaternion 回调尚未触发（或 topic 订阅与 g_sensor_data 赋值不在同一线程同步），读到的是初始值全零。

**绕过方案**：使用 `frame=0`（NED 直通模式）发速度指令，完全跳过四元数旋转。此时体坐标系方向无法保证正确，但水平移动指令仍会被执行。等起飞方案修复后可以试验确认方向漂移量级。

---

## 四、起飞方案修复（待充电完毕后执行）

### 4.1 PSDK 侧：flight_logic.c

`F_AttiCtrlTakeOff()` 当前逻辑：

```c
// 当前（有问题）
if(loopInput->flight_flag == 0) {
    if(Dji_FlightControlMonitoredTakeoff()) {
        loopInput->flight_flag = 1;
        // 即使在此处设置 VELOCITY 模式也无用——MonitoredTakeoff 已经锁定
    }
}
```

目标逻辑：

```c
// 目标（修复后）
if(loopInput->flight_flag == 0) {
    DjiFlightController_TurnOnMotors();

    T_DjiFlightControllerJoystickMode jm = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(jm);
    DjiFlightController_ObtainJoystickCtrlAuthority();

    loopInput->flight_flag = 1;
}
```

### 4.2 ROS 侧：test_flight.py

起飞后的全部运动（爬升、前进、后退、下降）由 `/flightByVel` 的 `frame=0`（NED 直通） 发速度指令完成，绕过四元数。

---

## 五、车-机状态机联动（待开发）

### 5.1 架构

```
智能车（上位机）                    机载盒子                    M3E
┌─────────────────┐   WiFi/ROS    ┌──────────────┐           ┌──────┐
│ 状态机控制器      │◄════════════►│ fcProxy+PSDK │◄─────────►│ 飞控  │
│                 │              │              │  USB Bulk │      │
│ state_machine.py│  /flightByVel│              │           │      │
│                 │  /uavdata    │              │           │      │
│                 │  /takeoff... │              │           │      │
└─────────────────┘              └──────────────┘           └──────┘
```

### 5.2 车-机状态机联动（已实现 `state_machine.py`，~390 行）

架构：**车辆（YL-R8, Jetson Nano）是主控端**，通过 WiFi TCP 发 JSON 指令。
**本工程（M3E 无人机）是执行端**，接收指令后执行飞行。

```
车辆 (YL-R8, Nano)                   本工程 (M3E, 上位机)
┌──────────────────┐   WiFi/TCP     ┌──────────────────────────┐
│ rescue_sm.py      │  JSON指令      │ state_machine.py          │
│                  │──────────────►│                          │
│ cmd: "takeoff"   │               │ → 起飞 → PD悬停           │
│ cmd: "fly_to"    │               │ → 飞向目标坐标            │
│ cmd: "land"      │               │ → 降落                   │
│                  │◄──────────────│                          │
│                  │  状态反馈JSON  │ {state, pos, health}     │
└──────────────────┘               └──────────────────────────┘
```

`state_machine.py` 核心接口：

```python
sm = DroneStateMachine(flight_ctrl, vo_fusion, position_ctrl)
# 车辆发来指令
sm.handle_car_command("takeoff", {"altitude": 0.5})
sm.handle_car_command("fly_to", {"x": 1.0, "y": 0.5, "z": 0.5})
sm.handle_car_command("land")
# 无人机上报状态
sm.get_status()  # → {drone_state, position, vo_healthy, ...}
```

状态转移：`IDLE → TAKING_OFF → HOVERING → MOVING_TO_TARGET → HOLDING → RETURNING → LANDING → DONE`
每步有 timeout/重试/异常路径，可随时 `abort()`。待起飞方案修复后联调。

### 5.3 开发阶段

| 阶段 | 内容 | 预估 |
|------|------|------|
| P1 | 修复起飞（TurnOnMotors + 手动速度控制） | 0.5 天 |
| P2 | 验证水平移动（NED 直通，frame=0） | 0.5 天 |
| P3 | 实现状态机控制器 state_machine.py | 1 天 |
| P4 | 车机联动联调（任务模拟） | 1 天 |
| P5 | AprilTag 精准降落集成 | 1 天 |

---

## 六、机载侧全部修改清单

### PSDK 侧

| 文件 | 行 | 改动 |
|------|-----|------|
| `application/sensor.h` | +2 | 加 positionVO + frame |
| `application/tta_fc_subscription.c` | +20 | 订阅 topic 41@50Hz |
| `gcs/gcs_transmit.cpp` | +6 | VO → param[0..3] |
| `gcs/gcs_receive.cpp` | +1 | param[1] → frame |
| `flight_control/flight_logic.c` | ~40 | F_velCtrlFlight 双模式 + F_AttiCtrlTakeOff 修复 |

### ROS 侧

| 文件 | 行 | 改动 |
|------|-----|------|
| `msg/uavdata.msg` | +4 | vo_x/y/z/health |
| `msg/flightByVel.msg` | +1 | uint8 frame |
| `fcProxy.cpp` | +10 | frame→param[1]; frame==2 位置偏移 |
| `uavData.h/.cpp` | +7 | 提取 VO |
| `publish.cpp` | +4 | 发布 VO |

### 未改动

- TTALINK 协议格式（param[4] 原本闲置，二进制兼容）
- `/takeoffOrLanding` / `/gimbalControl` 服务
- `rtsp_decode` 视频流
- 飞控自稳、避障、低电量保护

---

## 七、上位机模块

| 文件 | 行数 | 功能 |
|------|------|------|
| `vo_fusion.py` | 370 | VO+陀螺+AprilTag 融合定位引擎，全 PD 保护 |
| `telemetry.py` | 140 | /uavdata 订阅 + TelemetryFrame 解析 |
| `utils.py` | 230 | 坐标变换、GPS 计算、滤波器 |
| `flight_control.py` | 138 | /flightByVel(takeoff/land/gimbal，frame=0/1/2) |
| `path_executor.py` | 260 | PD 位置控制器：fly_to(), fly_path(), takeoff(), land() |
| `safety.py` | 140 | 起飞前检查、飞行中监控、hearthbeat/VO/高度 |
| `live_position.py` | 80 | 实时 VO 位置查看器 |
| `examples/takeoff_hover_land.py` | 60 | 起飞测试 |

---

## 八、风险与已知限制

| 风险 | 缓解 |
|------|------|
| MonitoredTakeoff joystick lock | → TurnOnMotors + 手动速度控制 |
| 四元数回调全零 | → NED 直通 (frame=0) 绕过 BODY 转换 |
| 飞控 arrested 状态 (HMS 0x16200301) | 拔电池 10 秒重置 |
| RC 不在 N 档 → joystick auth 失败 | 起飞前检查 RC 档位 |
| 遥控器摇杆偏移 → RC 自动接管 | 遥控器放桌面勿碰 |
| VO 室内纹理不足 → xHealth/yHealth=0 | 降低高度、增加地面标记 |
| 陀螺偏航漂移 | AprilTag 定时校正 |
| WiFi 高延迟/丢包 | 长命令走 nohup+日志文件 |

---

## 九、参考

- [ME3 项目文档分析](../ME3/ME3项目文档分析.md)
- [机载盒子修改记录](../ME3-deploy/onboard_box/MODIFICATIONS.md)
- [使用手册 README.md](./README.md)
- PSDK `dji_fc_subscription.h` — POSITION_VO (topic 41, line 487)
- PSDK `dji_flight_controller.h` — 坐标系枚举 / 控制 API
- PSDK `flight_logic.c` — F_velCtrlFlight() / F_AttiCtrlTakeOff()
- 机载盒子 `tta_fc_subscription.c` — 当前 topic 订阅清单
- 机载盒子 `gcs_transmit.cpp` — TTALINK feedback 打包
- `tta_odom.cpp` — 原始开发者 odom 测试（作者注明"积分误差太大"）
