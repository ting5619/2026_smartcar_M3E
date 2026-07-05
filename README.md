# ME3 无人机上位机控制 — 使用手册

> 通过 WiFi 远程控制 DJI Mavic 3E 无人机，室内不依赖磁罗盘。

---

## 硬件

| 设备 | 说明 |
|------|------|
| DJI Mavic 3E | 飞机，顶部 Type-C 口连接机载盒子 |
| 天途边缘计算单元 V2 | RK3588 机载盒子，Ubuntu 20.04 + ROS Noetic |
| 遥控器 | DJI RC，P 档（最上方） |
| 上位机 | Windows/Linux，连接同一 WiFi |

物理连接：

```
飞机顶部 Type-C ──── 盒子底部 Type-C 公头 (PSDK 口，直连供电+数据)
盒子调试口 (3针 TTL) ──── USB转TTL 线 ──── 电脑 (仅首次配网/故障时用)
盒子 WiFi ──── 路由器 ──── 上位机 WiFi (日常开发)
```

---

## 什么时候需要开什么

三种典型场景，供电方式和需要的设备不同。

### 场景 A：日常写代码、改上位机程序（最常用）

| 条件 | 状态 |
|------|------|
| 机载盒子 | **需要上电**（WiFi 已配好） |
| 无人机 | **不需要**（完全不用开机） |
| 遥控器 | **不需要** |
| 电脑↔盒子调试口 | **不需要**（WiFi + SSH 就够） |
| 电脑↔盒子 WiFi | **需要**（SSH 登录、传文件、跑 ROS 节点） |

```
电脑 ──WiFi──► 盒子 (SSH)
                │
                ├── roscore         ← 能跑
                ├── fcProxy         ← 能跑但 /uavdata 全为 0
                ├── 编译代码         ← 能跑
                └── 文件编辑         ← 能跑
```

可以做的事：
- SSH 登录盒子，编辑源码
- 编译 PSDK 或 ROS 包
- 启动 roscore + fcProxy，测试 ROS 通信链路
- 上位机连 ROS Master，验证 topic 能 list、消息格式正确

**无法做的事**：获取真实飞控数据（`/uavdata` 全 0）、飞行控制（飞控不响应）

### 场景 B：地面联调、读传感器、不飞

| 条件 | 状态 |
|------|------|
| 机载盒子 | **需要上电** |
| 无人机 | **需要开机 + 展开机臂**（自检完成） |
| 遥控器 | **需要开机**（P 档） |
| 电脑↔盒子调试口 | **不需要** |
| 电脑↔盒子 WiFi | **需要** |

```
飞机 (开机, 展开机臂)
  │ USB Type-C (PSDK口)
  ▼
盒子 (通过飞机供电)
  │ WiFi
  ▼
电脑 (SSH + ROS)
```

可以做的事：
- 启动 PSDK 服务 → 获取真实 `/uavdata`（GPS/IMU/姿态/高度/VO 位置）
- 测试飞控数据订阅（包括未来新增的 POSITION_VO topic 41）
- 调云台角度
- **电机不会转**（没有起飞指令）

**注意**：飞机需要先在室外记录 Home 点才能搬回室内联调。

### 场景 C：实际飞行

| 条件 | 状态 |
|------|------|
| 机载盒子 | **需要上电** |
| 无人机 | **需要开机 + 展开机臂 + Home 点已记录** |
| 遥控器 | **需要开机**（P 档，作为紧急接管备用） |
| 电脑↔盒子调试口 | **不需要** |
| 电脑↔盒子 WiFi | **需要** |

可以做的事：场景 B 的全部 + 起飞/降落/速度控制/路径飞行。

---

### 调试口的角色

| 阶段 | 需要 USB 转 TTL 线吗 |
|------|---------------------|
| 首次拿到盒子，配 WiFi | ✅ 需要（没有 WiFi 没法 SSH，只能串口） |
| WiFi 配好后，日常开发 | ❌ 不需要（WiFi + SSH） |
| WiFi 连不上了 | ✅ 需要（串口进去重配） |
| 烧录固件 | ❌ 用 Type-C 烧录口，不用串口 |

**结论**：WiFi 一旦配好，调试口就可以拔掉。只在 WiFi 出问题时才需要重新接上排障。

---

## 代码目录布局

### D:\ME3 — 官方资料（只读，不修改）

```
D:\ME3\
├── ME3项目文档分析.md          ← 项目文档分析
├── psdk/                       ← DJI PSDK v3.4.0 官方 SDK
│   ├── psdk_lib/include/       ← PSDK C/C++ API 头文件 (34个API模块)
│   ├── psdk_lib/lib/           ← 预编译库 (aarch64 / x86_64)
│   └── README.md               ← PSDK 说明
├── ROS接口程序/                 ← 接口说明 (README.txt)
├── 使用手册/                    ← 产品手册、烧录教程、配网说明
│   ├── 天途边缘计算单元机载版V2产品介绍(2).pdf
│   ├── 天途边缘计算单元机载版V2使用手册.docx
│   ├── 天途边缘计算单元机载版V2烧录教程.docx
│   ├── 新版机载算力盒子网络连接指令.md
│   ├── 新版机载算力盒子V2拓展内存教程.docx
│   ├── DriverAssitant_v5.1.1/  ← RK3588 烧录驱动
│   └── M3E飞控 (硬解码教育版）/  ← tta_m3e_rtsp ROS 包源码 (参考)
└── 固件/                        ← ok3588.img, rk3588_rtsp.img, parameter.txt
```

### D:\ME3-deploy — 我们的工程代码和工具（可修改）

```
D:\ME3-deploy\
├── README.md                   ← 使用手册 (本文件)
├── PLAN.md                     ← 实施计划 & 架构设计
├── requirements.txt            ← Python 依赖
├── me3_groundstation/          ← 上位机控制库
│   ├── connection.py           ← ROS 连接管理
│   ├── telemetry.py            ← 遥测数据订阅
│   ├── vo_fusion.py            ← VO 位置融合 (洗毒 + 陀螺 + AprilTag)
│   ├── flight_control.py       ← 飞行控制封装
│   ├── path_executor.py        ← 闭环路径执行器
│   └── safety.py               ← 安全监控
├── examples/                   ← 使用示例
├── paths/                      ← 预定义路径文件 (JSON)
├── scripts/                    ← 运维脚本
├── tools/                      ← 辅助工具 (串口助手、数据采集器等)
├── tests/                      ← 测试
└── onboard_box/                ← ★ 机载盒子修改过的源码副本
    ├── MODIFICATIONS.md        ← 修改说明（改了什么、为什么改）
    ├── psdk/                   ← PSDK 侧修改 (3 文件)
    ├── catkin_ws/              ← ROS 侧修改 (4 文件)
    └── _backup_originals/      ← 修改前的原始文件（可 diff）
```

### 机载盒子 — 关键代码路径（以 `forlinx` 身份通过 SSH 访问）

```
/home/forlinx/
├── psdk/                       ← PSDK 工程 (DJI官方，有本地修改)
│   ├── build/                  ← cmake 构建输出
│   │   └── bin/
│   │       ├── dji_sdk_demo_linux     ← PSDK C 版可执行文件
│   │       └── dji_sdk_demo_linux_cxx ← PSDK C++ 版可执行文件 ★
│   ├── samples/                ← PSDK 示例源码 ★
│   │   └── sample_c++/platform/linux/manifold2/
│   │       ├── application/    ← 主程序 + fc_subscription + flight_control
│   │       │   ├── main.cpp                    ← ★ 入口
│   │       │   ├── tta_fc_subscription.c/.h    ← ★ 飞控数据订阅 (需改)
│   │       │   ├── tta_flight_control.c/.h     ← ★ 飞控控制逻辑
│   │       │   ├── sensor.h                    ← ★ 传感器结构体 (需改)
│   │       │   ├── server_communication.cpp/.h ← PSDK↔TTALINK 通信
│   │       │   ├── stream_pusher.cpp/.h        ← RTSP 视频推流
│   │       │   ├── gimbalControl.cpp/.h        ← 云台控制
│   │       │   └── dji_sdk_app_info.h          ← ★ PSDK App Key 配置
│   │       ├── gcs/            ← TTALINK 收发 + 数据打包 ★
│   │       │   ├── gcs_transmit.cpp/.h         ← ★ update_ctrl_feed_back() (需改)
│   │       │   └── gcs.cpp/.h                  ← TTALINK 消息路由
│   │       ├── flight_control/ ← 飞控 PID 逻辑
│   │       ├── hal/            ← 硬件抽象 (USB Bulk / UART / 网口)
│   │       ├── proxy_src/      ← TCP/UDP/串口通信库
│   │       ├── ttalink_src/    ← TTALINK 编解码
│   │       ├── stream_pusher/  ← 视频流推送
│   │       ├── camera_manager/ ← 相机管理
│   │       └── pid/            ← PID 控制器
│   ├── psdk_lib/               ← PSDK 库文件 (头文件+预编译.a)
│   ├── ttalink/                ← TTALINK 协议定义 (100+ 消息类型)
│   └── tools/                  ← file2c 工具
│
├── catkin_ws/                  ← ROS 工作空间
│   ├── src/tta_m3e_rtsp/       ← ★ ROS 包源码
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/
│   │   │   ├── flightByVel.msg ← 速度控制消息 (需加 vo_* 字段)
│   │   │   └── uavdata.msg     ← 遥测消息 (需加 vo_* 字段)
│   │   ├── src/Controller/
│   │   │   ├── fcProxy.cpp     ← ★ PSDK↔ROS 桥接主程序
│   │   │   ├── uavData.cpp/.h  ← ★ PSDK 数据接收 (需改)
│   │   │   ├── publish.cpp/.h  ← ★ ROS topic 发布 (需改)
│   │   │   ├── tta_odom.cpp    ← 原始 odom 测试 (漂移严重，仅参考)
│   │   │   ├── proxy_src/      ← TCP/UDP 通信库
│   │   │   ├── ttalink/        ← TTALINK 消息编解码
│   │   │   └── utils/          ← 工具
│   │   ├── scripts/
│   │   │   ├── tta_apriltag_detect.py ← ★ AprilTag 视觉伺服
│   │   │   ├── uav.py          ← 起飞+速度控制示例
│   │   │   ├── flight_node.py  ← 比赛飞行节点
│   │   │   └── apriltag_m3e.py ← AprilTag M3E 版
│   │   ├── launch/             ← ROS launch 文件
│   │   │   ├── startup_ctl.launch  ← ★ 常用来启动 fcProxy
│   │   │   └── tta.launch      ← 视觉+飞行联动
│   │   ├── cfg/                ← 动态参数 (PID, 状态机)
│   │   ├── srv/                ← ROS 服务定义
│   │   └── rknn_model/         ← YOLOv5 RKNN 模型
│   ├── build/                  ← catkin build 输出
│   └── devel/                  ← catkin devel (编译产物)
│       └── lib/tta_m3e_rtsp/
│           ├── fcProxy         ← ★ 已编译节点
│           └── uavNav          ← 已编译节点
│
├── Desktop/
│   └── dji_sdk_demo_on_jetson_cxx  ← 旧版 PSDK (jetson 移植版)
│
└── /opt/ttaviation/            ← 天途闭源组件
    ├── OK3588-usb-device-mode/ ← USB gadget 启动脚本 + startup_bulk
    ├── deep/                   ← DeepStream/视觉加速库
    └── mylib/                  ← TTA 闭源库 (RTSP, 视频中心等)
```

**★ 标记** = 我们的 VO 方案需要改动的文件。

---

## 环境

### 机载盒子

| 项目 | 值 |
|------|-----|
| 主机名 | `ok3588` |
| 用户名 | `forlinx` |
| 密码 | `forlinx` |
| 系统 | Ubuntu 20.04 LTS, aarch64 |
| ROS | Noetic (`/opt/ros/noetic`) |
| 工作空间 | `~/catkin_ws` |
| PSDK 程序 | `~/psdk/build/bin/dji_sdk_demo_linux_cxx` |

### 上位机

- Python 3.8+ + `rospy` + `paramiko` (SSH)
- 与盒子同一 WiFi 网段
- 工程代码在 `D:\ME3-deploy\`

---

## 启动流程

### 1. 飞机上电

```
装电池 → 开机 → 展开机臂 → 等自检完成 (尾灯变绿或黄)
遥控器开机 → P 档
需在室外记录 Home 点后才能搬回室内
```

### 2. 机载盒子启动服务

```bash
# SSH 登录
ssh forlinx@<盒子IP>

# 按顺序启动三个进程：

# ① PSDK 服务 (需 root，因为 Logs 目录权限)
sudo /home/forlinx/psdk/build/bin/dji_sdk_demo_linux_cxx &

# ② ROS Master
source /opt/ros/noetic/setup.bash
roscore &

# ③ fcProxy (PSDK ↔ ROS 桥接)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch tta_m3e_rtsp startup_ctl.launch &
```

验证：

```bash
rostopic list
# 应看到: /uavdata, /flightByVel, /takeoffOrLanding, /gimbalControl

rostopic echo /uavdata -n 1
# latit/longi/altit/velN/velE/velD 应有非零值
```

### 3. 上位机连接

```bash
export ROS_MASTER_URI=http://<盒子IP>:11311
export ROS_IP=<上位机IP>

rostopic list  # 应看到盒子上的所有 topic
```

---

## ROS 接口速查

| 接口 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/uavdata` | Topic (10Hz) | 盒子→上位机 | 遥测: GPS/IMU/姿态/VO位置/速度/高度 |
| `/flightByVel` | Topic | 上位机→盒子 | 速度指令 {vel_n, vel_e, vel_d, targetYaw, fly_time, frame} |
| `/takeoffOrLanding` | Service | 上位机→盒子 | 起飞(1) / 降落(2) |
| `/gimbalControl` | Service | 上位机→盒子 | 云台角度 {pitch, roll, yaw} |

### flightByVel 消息（发送速度指令）

```
float32 vel_n       # 向前速度 (NED: North, 或 BODY: Forward)
float32 vel_e       # 向右速度
float32 vel_d       # 垂直速度 (正值=下降! 上升用负值)
float32 targetYaw   # 目标偏航角 (deg, NED 模式)
float32 fly_time    # 指令持续时间 (s)
uint8 frame         # 0=NED, 1=BODY (室内必须=1!)
```

**室内飞行必须设置 `frame=1`（BODY 坐标系），不依赖磁罗盘**。

### uavdata 消息（接收遥测反馈）

```
float64 latit, longi     # WGS84 经纬度 (deg)
float32 altit            # 海拔高度 (m)
float32 velN, velE, velD # NED 速度 (m/s)
float32 atti_pitch/roll/yaw  # 姿态角 (deg)
float32 gyro_pitch/roll/yaw  # 角速度 (rad/s)
float32 accN, accE, accD     # 加速度 (m/s², 注意这是 raw 不是 ground)
float32[] quat           # 四元数 [w, x, y, z]

# ▼ 以下为新增 VO 位置字段 (来自 PSDK Topic 41)
float32 vo_x             # 全向视觉定位 x (m, NED标注)
float32 vo_y             # 全向视觉定位 y (m)
float32 vo_z             # 全向视觉定位 z (m, Down方向)
uint8 vo_health          # bit0=xHealth, bit1=yHealth, bit2=zHealth
```

---

## 坐标系

### NED (室外 GPS 可用)

| 轴 | 方向 | 正值 |
|----|------|------|
| X | 北 | 向北 |
| Y | 东 | 向东 |
| Z | 下 | **下降 (上升用负值!)** |

### BODY (室内)

| 轴 | 方向 | 正值 |
|----|------|------|
| X | 机头前方 | 前进 |
| Y | 机身右侧 | 右移 |
| Z | 下 | 下降 |

**室内默认用 BODY 坐标系，不依赖磁罗盘。**

---

## 程序自动飞行

### 完整指令流程

```
STEP 1 ──── 上位机 ──► /takeoffOrLanding(1) ──► fcProxy ──► PSDK ──► Dji_FlightControlMonitoredTakeoff()
                                                                    │  自动爬升至 ~1.2m
                                                                    │  joystick 重置为 VELOCITY 模式
                                                                    │  ← ★ 已修复：MonitoredTakeoff 完成后自动解锁 z 轴
STEP 2 ──── 上位机 ──► /flightByVel {frame=1, vel_n=-0.3} ──► 向后微移(打破悬停惯性)
STEP 3 ──── 上位机 ──► /flightByVel {frame=1, vel_d=+0.3} ──► 下降至目标高度
                                                                    │  Dji_FlightController_ExecuteJoystickAction()
                                                                    │  vel_d 正值 = Down
STEP 4 ──── 上位机 ──► /flightByVel {frame=1, vel_d=0} ──► 悬停保持
STEP 5 ──── 上位机 ──► /takeoffOrLanding(2) ──► 降落
```

### `flightByVel` 的 `frame` 字段三种模式

| frame | 模式 | 飞控调用 | vel_n/vel_e 含义 | vel_d 含义 |
|-------|------|---------|-----------------|------------|
| 0 | NED 速度 | `Dji_FlightControlVelocityAndYawRateCtrl` | North/East m/s | Down m/s |
| **1** | **BODY 速度** | `Dji_FlightControlVelocityAndYawRateCtrl` | **Forward/Right m/s** | **Down m/s** |
| 2 | NED 位置偏移 | `Dji_FlightControlMoveByPositionOffset` | North/East m | Down m |

**室内自动飞行默认使用 frame=1（BODY 速度），不依赖磁罗盘。**

### 前置条件

1. 飞机电池装入 → 开机 → **展开机臂** → 窗边等待 GPS 锁定 Home 点（尾灯闪绿）
2. 保持电池不拔，搬到测试场地
3. 遥控器开机 P 档（紧急备用，推油门即可接管）
4. 盒子启动三个服务：
   ```bash
   sudo /home/forlinx/psdk/build/bin/dji_sdk_demo_linux_cxx &
   source /opt/ros/noetic/setup.bash && roscore &
   source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch tta_m3e_rtsp startup_ctl.launch &
   ```
5. 上位机设置 `ROS_MASTER_URI=http://172.20.10.6:11311`

### 快速起飞测试

**在盒子上直接跑**（SSH 进去）：

```bash
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
python3 ~/test_flight.py
```

**从上位机发指令**（需安装 ROS Noetic）：

```python
from me3_groundstation.flight_control import ME3FlightController
import time

ctrl = ME3FlightController()

# 起飞
ctrl.takeoff()
time.sleep(8)  # 等 MonitoredTakeoff 自动完成 + joystick 模式重置

# 向后微移（打破悬停惯性，同时验证水平控制）
for _ in range(5):
    ctrl.set_velocity_body(-0.3, 0, 0, frame=1)  # BODY, 后退
    time.sleep(0.1)
ctrl.hover(1.0)

# 下降至目标高度（vel_d 正值 = Down）
for _ in range(30):
    ctrl.set_velocity_body(0, 0, +0.3, frame=1)  # BODY, 下降
    time.sleep(0.1)

# 悬停保持
ctrl.hover(5.0)

# 降落
ctrl.land()
```

### 实飞验证记录（2026-07-04）

| 测试 | 起飞 | 高度 | 下降 | 结论 |
|------|:--:|------|:--:|------|
| v2~v5 | ✅ | ~110cm | ❌ | MonitoredTakeoff z-lock 导致下降指令被拒 |
| v6~v9 | ❌ | 0cm | — | 电池低压保护 |
| v10 | **待测** | — | — | 已修复：MonitoredTakeoff 后自动重置 joystick 为 VELOCITY 模式 |

### 已修复的 PSDK 改动

`flight_logic.c` → `F_AttiCtrlTakeOff()`：
```c
// MonitoredTakeoff 完成后，飞控内部锁定在 POSITION 模式
// 必须手动重置为 VELOCITY 模式，后续速度指令才会生效
if(Dji_FlightControlMonitoredTakeoff()) {
    loopInput->flight_flag = 1;
    T_DjiFlightControllerJoystickMode jm = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(jm);
}
```

### 关键注意事项

1. `frame` 室内必须 =1（BODY）。frame=0 依赖罗盘，室内方向有毒
2. `vel_d` 正值=下降。neg 值=上升（如 `vel_d=-0.3`）
3. 起飞后至少等 **8 秒**（MonitoredTakeoff 爬升 + joystick 模式重置）
4. 第一次指令建议**向后微移** 0.3m/s × 0.5s（打破悬停惯性）
5. 遥控器 P 档随时可接管——推油门立即交还控制权给 RC

### 安全

- **Home 点**：需要 GPS 锁定后才能记录。窗边开机锁点后保持电池不拔即可搬回室内。无 GPS 时也能起飞但无法 RTH
- 遥控器推油门 → 上位机指令立即失效
- PSDK 失去心跳 3s → 飞控自动恢复 RC 控制
- 电池 < 15% → 飞控强制降落（DJI 内置，不可覆盖）

---

## WiFi 配网

盒子默认连 `霆` (5GHz)。如需更换：

```bash
# 扫描
iw dev wlan0 scan | grep SSID

# 配网
wpa_passphrase "<SSID>" "<密码>" > /etc/wpa_supplicant.conf
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant.conf
dhclient wlan0
```

如果 SSID 含中文，用 hex 编码：

```bash
echo "<base64编码的配置>" | base64 -d > /etc/wpa_supplicant.conf
```

---

## 串口调试

盒子 3 针调试口（靠近烧录口）：

| 针脚 | 连接 |
|------|------|
| 靠近烧录口 | GND |
| 中间 | 盒子 RX ← 电脑 TX |
| 远离烧录口 | 盒子 TX → 电脑 RX |

电脑端：MobaXterm/Putty，波特率 115200，3.3V TTL。

---

## 固件烧录

**仅在首次部署或系统损坏时需要。**

1. 安装驱动：`使用手册/DriverAssitant_v5.1.1/DriverInstall.exe`
2. 按住盒子 RECOVERY 白色按键 → 给盒子上电 → 松手
3. 烧录软件选 `boot.img` + `ok3588.img` → 烧录

详见 `D:\ME3\使用手册\天途边缘计算单元机载版V2烧录教程.docx`

---

## 故障排查

| 现象 | 检查 |
|------|------|
| PSDK 报 `Core init error` | 飞机未开机 / USB 未识别 / 机臂未展开 |
| PSDK 报 `File system init error` | 用 `sudo` 运行（Logs/ 目录权限） |
| `/uavdata` 全为 0 | PSDK 未启动 / fcProxy 未启动 / 飞机 USB 未通 |
| SSH `Connection refused` | 盒子 WiFi 未连 / IP 变了 |
| `rostopic list` 报 `Unable to communicate with master` | roscore 未运行 / ROS_MASTER_URI 不对 |
| `wpa_supplicant` 解析失败 | SSID 含中文需用 hex 编码 |
| 盒子 `fc000000.usb: not attached` | 飞机自检未完成 / USB 线接触不良 |

---

## 参考

- [ME3 项目文档分析](../ME3/ME3项目文档分析.md)
- [实施计划 PLAN.md](./PLAN.md)
- [机载盒子网络连接指令](../ME3/使用手册/新版机载算力盒子网络连接指令.md)
- [天途边缘计算单元使用手册](../ME3/使用手册/天途边缘计算单元机载版V2使用手册.docx)
- [天途边缘计算单元烧录教程](../ME3/使用手册/天途边缘计算单元机载版V2烧录教程.docx)
