# 机载盒子代码修改记录

> 目的：新增 PSDK Topic 41 (POSITION_VO) 全向视觉定位数据 → ROS `/uavdata`  
> 改动量：7 个文件，约 40 行  
> 日期：2026-07-04

---

## 数据流

```
DJI 飞控 (全向视觉+超声波+IMU+红外TOF融合)
    │ POSITION_VO Topic 41, 50Hz
    ▼
① sensor.h             ← 新增字段记录原始数据
② tta_fc_subscription.c ← 订阅 topic 41 + 读取 + 赋值到 sensor
③ gcs_transmit.cpp      ← 打包到 TTALINK feedback 的 param[0..3]
    │ TTALINK 消息 8201 (已有消息, param[4] 原为预留字段)
    ▼
④ uavdata.msg           ← 新增 vo_x, vo_y, vo_z, vo_health 字段
⑤⑥ uavData.h/.cpp       ← 从 feedback.param[] 提取 VO 数据
⑦ publish.cpp            ← 发布到 ROS /uavdata topic
    │
    ▼
上位机 me3_groundstation/vo_fusion.py
```

---

## 修改清单

### PSDK 侧（3 文件）

机载路径前缀：`/home/forlinx/psdk/samples/sample_c++/platform/linux/manifold2/`

| # | 文件 | 行 | 改动 |
|---|------|-----|------|
| ① | `application/sensor.h` | +1 | 结构体 `sensor_t` 新增 `T_DjiFcSubscriptionPositionVO positionVO;` |
| ② | `application/tta_fc_subscription.c` | +20 | `Dji_FcSubscriptionStartService()` 订阅 topic 41 @50Hz；`UserFcSubscription_Task()` 读取最新值；赋值到 `g_sensor_data.positionVO` |
| ③ | `gcs/gcs_transmit.cpp` | +6 | `update_ctrl_feed_back()` 中打包 `sensor->positionVO.{x,y,z,xHealth,yHealth,zHealth}` 到 `sd_msg.param[0..3]` |

### ROS 侧（4 文件）

机载路径前缀：`/home/forlinx/catkin_ws/src/tta_m3e_rtsp/`

| # | 文件 | 行 | 改动 |
|---|------|-----|------|
| ④ | `msg/uavdata.msg` | +4 | 新增字段 `float32 vo_x, vo_y, vo_z, uint8 vo_health` |
| ⑤ | `src/Controller/uavData.h` | +3 | 反馈结构体新增 `float vo_x, vo_y, vo_z; uint8_t vo_health;` |
| ⑥ | `src/Controller/uavData.cpp` | +4 | 从 `feedback_data->param[0..3]` 提取 VO 数据 |
| ⑦ | `src/Controller/publish.cpp` | +4 | 将 VO 数据拷贝到 `/uavdata` 消息 |

### 未改动的部分

- TTALINK 协议格式（`param[4]` 原本为预留字段，二进制兼容）
- fcProxy 消息转发逻辑
- `/flightByVel` 速度控制接口
- `/takeoffOrLanding` / `/gimbalControl` 服务
- `rtsp_decode` 视频流

---

## 机载盒子完整工程树（标注修改位置）

```
/home/forlinx/
├── psdk/                                    ← PSDK 工程 (DJI 官方 + 我们的修改)
│   ├── build/bin/dji_sdk_demo_linux_cxx     ← 编译输出 ★
│   ├── samples/sample_c++/platform/linux/manifold2/
│   │   ├── application/
│   │   │   ├── main.cpp                     ← 入口
│   │   │   ├── sensor.h                     ← ① 已修改 ★
│   │   │   ├── tta_fc_subscription.c        ← ② 已修改 ★
│   │   │   ├── tta_fc_subscription.h
│   │   │   ├── tta_flight_control.c/.h      ← 飞控逻辑
│   │   │   ├── server_communication.cpp/.h  ← PSDK↔TTALINK 通信
│   │   │   ├── stream_pusher.cpp/.h         ← RTSP 推流
│   │   │   ├── gimbalControl.cpp/.h         ← 云台控制
│   │   │   └── dji_sdk_app_info.h           ← PSDK App Key
│   │   ├── gcs/
│   │   │   ├── gcs_transmit.cpp             ← ③ 已修改 ★
│   │   │   ├── gcs_transmit.h
│   │   │   └── gcs.cpp/.h                   ← TTALINK 消息路由
│   │   ├── flight_control/                  ← 飞控 PID
│   │   ├── hal/                             ← USB Bulk / UART / 网口
│   │   ├── proxy_src/                       ← TCP/UDP/串口库
│   │   ├── ttalink_src/                     ← TTALINK 编解码
│   │   ├── stream_pusher/                   ← 视频推流
│   │   ├── camera_manager/                  ← 相机管理
│   │   └── pid/                             ← PID 控制器
│   ├── psdk_lib/                            ← PSDK 头文件+预编译库
│   └── ttalink/                             ← TTALINK 协议定义
│
├── catkin_ws/                               ← ROS 工作空间
│   ├── src/tta_m3e_rtsp/                    ← ROS 包源码
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/
│   │   │   ├── uavdata.msg                  ← ④ 已修改 ★
│   │   │   └── flightByVel.msg
│   │   ├── src/Controller/
│   │   │   ├── fcProxy.cpp                  ← PSDK↔ROS 桥接主程序
│   │   │   ├── uavData.h                    ← ⑤ 已修改 ★
│   │   │   ├── uavData.cpp                  ← ⑥ 已修改 ★
│   │   │   ├── publish.h
│   │   │   ├── publish.cpp                  ← ⑦ 已修改 ★
│   │   │   ├── tta_odom.cpp                 ← 原始 odom 测试(参考)
│   │   │   ├── proxy_src/                   ← TCP/UDP 通信库
│   │   │   ├── ttalink/                     ← TTALINK 编解码
│   │   │   └── utils/                       ← 工具
│   │   ├── scripts/                         ← Python 脚本
│   │   ├── launch/                          ← ROS launch 文件
│   │   ├── cfg/                             ← 动态参数
│   │   └── srv/                             ← 服务定义
│   ├── build/                               ← catkin build
│   └── devel/lib/tta_m3e_rtsp/
│       ├── fcProxy                          ← 编译产物 ★
│       └── uavNav
│
└── /opt/ttaviation/                         ← 天途闭源组件 (不改)
    ├── OK3588-usb-device-mode/              ← USB gadget 启动脚本
    ├── deep/                                ← DeepStream 视觉库
    └── mylib/                               ← TTA 闭源库
```

---

## uavdata.msg 新增字段

```msg
# VO fused position (topic 41) from PSDK
float32 vo_x        # x 坐标 (NED标注, 飞控内部fusion, 单位: m)
float32 vo_y        # y 坐标 (NED标注)
float32 vo_z        # z 坐标 (Down方向, 单位: m)
uint8 vo_health     # bit0=xHealth, bit1=yHealth, bit2=zHealth
```

## TTALINK param[4] 编码

```
param[0] = positionVO.x        (float, m)
param[1] = positionVO.y        (float, m)
param[2] = positionVO.z        (float, m, Down)
param[3] = xHealth | (yHealth<<1) | (zHealth<<2)   (cast to float)
```
