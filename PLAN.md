# ME3 无人机上位机运动路径控制 — 实施计划

> 项目代号: ME3-deploy  
> 目标: 上位机通过 WiFi 控制 DJI Mavic 3E 无人机自动飞行（室内）  
> 约束: 机载侧仅最小化修改（PSDK 9 文件 ≈ 50 行），上位机自由开发  
> 日期: 2026-07-04

---

## 一、已完成 vs 待完成

### ✅ 已完成

| 类别 | 内容 | 状态 |
|------|------|------|
| **位置数据源** | PSDK Topic 41 (POSITION_VO) 订阅 → TTALINK → ROS /uavdata | ✅ 已验证 |
| **数据通路** | 7 文件修改，编译通过，飞行中持续输出 vo_x/y/z/health | ✅ 已验证 |
| **BODY/NED 切换** | flightByVel 新增 `frame` 字段，PSDK 侧 `F_velCtrlFlight()` 双模式 | ✅ 已实施 |
| **测试验证** | 室内静止 VO 漂移 <1cm/70s；室外飞行 VO health=3 全程保持 | ✅ 已验证 |
| **上位机库** | vo_fusion.py, telemetry.py, flight_control.py, path_executor.py 已就绪 | ✅ 已编写 |
| **测试脚本** | /tmp/test_flight.py (起飞20cm→悬停2s→降落) | ✅ 已部署 |

### ⏳ 待完成

| 内容 | 说明 |
|------|------|
| 实飞测试 | 明天充电完毕执行 test_flight.py |
| 闭环路径飞行 | 上位机 VO 反馈 + PID 控制 |

---

## 二、位置数据源 — Topic 41 (POSITION_VO)

### 2.1 由来

M3E 飞控内部已完成 **全向视觉 + 超声波 + IMU + 气压计 + 红外 TOF** 多传感器融合，输出为 PSDK Topic 41 (`DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO`)。这在 `dji_fc_subscription.h:487` 定义。

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

## 三、程序自动飞行 — 控制链路

### 3.1 指令流

```
上位机 me3_groundstation
  │ flight_control.py → set_velocity_body(vx, vy, vz, frame=1)
  │ path_executor.py  → PID 闭环用 VO 位置反馈
  ▼ ROS Topic /flightByVel
fcProxy.cpp → TTALINK 8200 {velN, velE, velD, atti_yaw, param[0]=fly_time, param[1]=frame}
  ▼ TCP 127.0.0.1:10086
gcs_receive.cpp → loopInput->frame = param[1]
  │ SetFlightCtrlSta(ROS_F_GPS_POS_VEL_ALTI_VEL)
  ▼
flight_logic.c → F_velCtrlFlight()
  ├─ frame==0: NED 直通
  └─ frame!=0: vel_body → vector_XYZ2NED_Quaternion() → Dji_FlightControlVelocityAndYawRateCtrl()
  ▼
DJI 飞控 → 电机
```

### 3.2 BODY/NED 切换开关

`flightByVel` 消息新增 `uint8 frame` 字段：

| frame 值 | 模式 | 速度含义 | 使用场景 |
|----------|------|---------|---------|
| 0 | NED | vel_n=北向, vel_e=东向 | 室外 GPS |
| 1 (默认) | BODY | vel_n=机头前, vel_e=机身右 | **室内** |

PSDK `F_velCtrlFlight()` 根据 `frame` 分支：
- `frame==0`：速度直通 joystick（原有行为，不动）
- `frame!=0`：体坐标系速度 → quaternion 旋转 → NED 速度 → joystick（新增）

### 3.3 不改的控制链路

- 遥控器 → DJI 飞控（最高优先级）✅
- `/takeoffOrLanding` 服务 ✅
- `/gimbalControl` 服务 ✅
- 飞控自稳、避障、低电量保护 ✅

---

## 四、机载侧全部修改清单

### PSDK 侧（6 文件）

| 文件 | 行 | 改动 |
|------|-----|------|
| `application/sensor.h` | +2 | 加 `T_DjiFcSubscriptionPositionVO positionVO;` 和 `uint8_t frame;` |
| `application/tta_fc_subscription.c` | +20 | 订阅 topic 41@50Hz + 读取 + 赋值 |
| `gcs/gcs_transmit.cpp` | +6 | VO 打包到 param[0..3] |
| `gcs/gcs_receive.cpp` | +1 | param[1] → loopInput->frame |
| `flight_control/flight_logic.c` | ~30 | F_velCtrlFlight() 双模式重写 |
| `gcs/gcs_receive.h` | 0 | 无需改动 |

### ROS 侧（3 文件）

| 文件 | 行 | 改动 |
|------|-----|------|
| `msg/uavdata.msg` | +4 | 加 vo_x, vo_y, vo_z, vo_health |
| `msg/flightByVel.msg` | +1 | 加 uint8 frame |
| `src/Controller/fcProxy.cpp` | +1 | param[1] = msg->frame |
| `src/Controller/uavData.h` | +3 | 加 vo 成员 |
| `src/Controller/uavData.cpp` | +4 | 从 param 提取 VO |
| `src/Controller/publish.cpp` | +4 | 拷贝到 uavdata |

### 不改的部分

- `fcProxy.cpp` 消息转发核心逻辑
- `tta_flight_control.c` — 飞控初始化、控制权获取
- `gcs.cpp` — TTALINK 消息路由
- TTALINK 协议格式（param[4] 原为预留字段，二进制兼容）
- `/takeoffOrLanding`、`/gimbalControl`、`rtsp_decode`

---

## 五、上位机模块

### 已创建

| 文件 | 行数 | 功能 |
|------|------|------|
| `me3_groundstation/__init__.py` | 8 | 包入口 |
| `me3_groundstation/vo_fusion.py` | 370 | VO+陀螺+AprilTag 融合定位引擎 |
| `me3_groundstation/telemetry.py` | 140 | /uavdata 订阅 + 解析为 TelemetryFrame |
| `me3_groundstation/utils.py` | 230 | 坐标变换(NED↔ENU↔BODY)、GPS计算、滤波器 |
| `me3_groundstation/flight_control.py` | 130 | 封装 /flightByVel + /takeoffOrLanding + /gimbalControl，自动判断 frame=1(BODY) vs frame=0(NED) |
| `me3_groundstation/live_position.py` | 80 | 实时位置查看器 |
| `me3_groundstation/path_executor.py` | 170 | PID 闭环路径执行器，VO 位置反馈驱动体坐标系速度 |
| `examples/takeoff_hover_land.py` | 60 | 起飞→悬停→降落测试脚本 |

### 接口速查

```python
from me3_groundstation.flight_control import ME3FlightController
from me3_groundstation.vo_fusion import VOFusion
from me3_groundstation.path_executor import PathExecutor

# 飞行控制
ctrl = ME3FlightController()
ctrl.takeoff()                           # 起飞
ctrl.land()                              # 降落
ctrl.set_velocity_body(1.0, 0, 0, frame=1)  # 体坐标 前进1m/s
ctrl.hover(2.0)                          # 悬停2秒

# VO 定位（不依赖罗盘）
vo = VOFusion()
vo.start()
vo.feed_vo(vx, vy, vz, health, t)        # 喂 VO 数据
vo.feed_quaternion(q0,q1,q2,q3,t)        # 喂姿态
vo.feed_gyro(gx, gy, gz, t)              # 喂陀螺
vo.correct_by_apriltag(tag_x, tag_y)     # 视觉校正
x, y, z = vo.get_position()              # 获取位置

# 闭环路径执行
executor = PathExecutor(ctrl, vo)
executor.takeoff_and_hover(0.2, 2.0)     # 起飞20cm悬停2s
executor.move_relative_body(1.0, 0)      # 前进1m（闭环）
executor.land()                           # 降落
```

---

## 六、测试结果

### VO 室内验证（2026-07-04）

| 测试 | 结果 |
|------|------|
| 静止 70s 漂移 | 0.1cm (net) |
| 静止噪声 | x: 0.8cm σ, y: 0.9cm σ |
| 搬运 75cm 检测 | VO x 跟踪到 -0.75m，清晰可辨 |
| VO health 全程 | 100% (3 = xH✅ yH✅ zH❌) |
| 初始化时间 | ~3s（30 frames） |

### VO 室外飞行验证（2026-07-04）

| 测试 | 结果 |
|------|------|
| 飞行时长 | 约 14 分钟 |
| VO health | 100% (全程 =3) |
| 低空降落后 VO 稳定性 | 波动 < 2cm |
| 兼容性 | 遥控飞行期间 VO 数据正常推送，不影响飞行 |

---

## 七、风险与已知限制

| 风险 | 缓解 |
|------|------|
| VO 纹理不足 → xHealth/yHealth=0 | 降低高度、增加地面纹理标记、降级为纯惯性+AprilTag |
| 陀螺偏航漂移 (~1°/min) | AprilTag 定时校正航向 |
| 室内罗盘不准 → NED 不可用 | **默认 BODY 坐标系**，不依赖罗盘 |
| WiFi 延迟导致位姿与指令不同步 | 时间戳对齐，VO 外推 |
| 遥控器优先级 > PSDK | 监听到 RC 接管立即停止发指令 |
| 当前 /flightByVel 的 frame 字段只对 vel 控制有效 | 起降不受影响；如需 NED 定点飞行需额外开发 |

---

## 八、参考

- [ME3 项目文档分析](../ME3/ME3项目文档分析.md)
- [机载盒子修改记录](../ME3-deploy/onboard_box/MODIFICATIONS.md)
- PSDK `dji_fc_subscription.h` — POSITION_VO (topic 41, line 487)
- PSDK `dji_flight_controller.h` — 坐标系枚举 (line 215-218)
- `tta_odom.cpp` — 原始开发者 odom 测试
- 机载盒子 `tta_fc_subscription.c` — 订阅清单
- 机载盒子 `flight_logic.c` — F_velCtrlFlight() 双模式实现
