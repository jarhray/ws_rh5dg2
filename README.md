# RH5DG2 ROS 2 工作区

RH5DG2 灵巧手的 13 通道串口驱动，运行于 Ubuntu 22.04 / ROS 2 Humble。提供角度、速度、力、电流和模式控制，状态与触觉反馈，以及 Python 客户端。

## 构建和启动

```bash
cd /home/jerry/dex_dlo/ws_rh5dg2
./scripts/build.sh
source install/setup.bash
ros2 launch rh5dg2_bringup hand.launch.py side:=left port:=/dev/ttyUSB0
```

右手使用 `side:=right`。双手需要两个独立串口：

```bash
ros2 launch rh5dg2_bringup dual.launch.py left_port:=/dev/ttyUSB0 right_port:=/dev/ttyUSB1
```

连接参数保存在 [`left.yaml`](src/rh5dg2_bringup/config/left.yaml) 和 [`right.yaml`](src/rh5dg2_bringup/config/right.yaml)：默认 115200、8N1、ID 1，首次有效反馈后默认接收命令；可用 `enable_write:=false` 启动为只读模式。显式 launch 参数覆盖 YAML，未传入时沿用 YAML。默认 `/dev/rh5dg2_left`、`/dev/rh5dg2_right` 是预留别名，需自行建立或改成实际设备路径。其他配置见 [启动说明](src/rh5dg2_bringup/README.md)。

新机器先安装 Humble、colcon、rosdep，再运行 `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble`。构建脚本使用系统 Python；运行建议使用未激活 Conda 的终端，不要混用原厂工作区。

**本次更新需统一重建：** 删除了 `transport`、`hardware_profile`、`serial_read_mode`、`angle_limit_mode`、`angle_min_raw`、`angle_max_raw` 参数，以及 `HandState.hardware_profile` 字段和 `diagnostics` 话题。移除旧启动命令中的这些参数，重新构建、source，并重启驱动和全部客户端。reconnect 不会加载新代码或 YAML。

## 读取和控制

另开终端，先 `source install/setup.bash`：

```bash
ros2 run rh5dg2_py hand_cli --namespace /inspire/left state
ros2 topic echo /inspire/left/angle_actual --once --qos-durability transient_local
```

最直接的角度示例：修改 [`angle_demo.py`](src/rh5dg2_py/rh5dg2_py/angle_demo.py) 顶部 `TARGET_ANGLES`，运行 `ros2 run rh5dg2_py angle_demo`。示例只向 `/inspire/left/angle_set` 发布一次 `JointCommand`，不读状态、不调用服务、不使能/禁用、不等待到位、不操作急停。退出仅释放 ROS 资源。

在已有 ROS 节点中可以直接发布（publisher 建议在节点初始化时创建）：

```python
from rh5dg2_interfaces.msg import JointCommand

publisher = node.create_publisher(JointCommand, '/inspire/left/angle_set', 1)
# 需要运动时，在订阅连接建立后发布：
message = JointCommand()
stamp = node.get_clock().now()
message.header.stamp = stamp.to_msg()
message.command_id = stamp.nanoseconds
message.values = [-1, -1, -1, -1, 1600, -1, -1, -1, -1, -1, -1, -1, -1]
publisher.publish(message)
```

驱动统一限幅并执行，处理结果发布到 `command_result`，实际角度发布到 `angle_actual`。是否查看反馈由调用者决定。需要服务应答时也可使用 `HandClient` / `hand_cli`，见 [Python 说明](src/rh5dg2_py/README.md)。

## 通道和范围

角度使用 **0.1 度的原始寄存器值**，例如 `1700` 表示 `170.0°`。驱动对所有角度入口统一截断到下表范围；定义在 [`registers.hpp`](src/rh5dg2_driver/include/rh5dg2_driver/registers.hpp) 的 `kAngleMin` / `kAngleMax`，当前根部范围已按试运行调整，端点仍需继续核对。

| 下标 | 机械含义 | 角度范围 |
| --- | --- | --- |
| 0 | 小指根部 | 1000–1650 |
| 1 | 无名指根部 | 1000–1650 |
| 2 | 中指根部 | 1000–1650 |
| 3 | 中指侧摆 | -150–150 |
| 4 | 食指根部 | 1000–1650 |
| 5 | 食指侧摆 | -150–150 |
| 6–9 | 小指、无名指、中指、食指第一弯曲 | 1050–1850 |
| 10 | 拇指侧摆 | 600–1700 |
| 11 | 拇指第一弯曲 | 1220–1600 |
| 12 | 拇指第二弯曲 | 1450–2040 |

例如食指根部请求 `1800` 时下发 `1650`。成功响应的 `applied_values` 是实际下发目标，`clamped` 标记被截断的通道；这两个字段不代表机械关节已到位。`-1` 始终表示不写该通道，也不能用于表示侧摆 -0.1°。全部跳过时不写串口；部分控制逐个写指定通道，不读回或重发其他通道的原目标。

速度范围 0–2500，力阈值 0–1000，电流限制 0–1500 mA；速度、力使用厂商原始标度。模式 0 为速度力保护，1 为力闭环，中指 2/3 和食指 4/5 必须成对一致。这些设置越界会拒绝，均要求 13 个整数。侧摆力阈值以对应根部通道为准；力反馈 6–9 没有对应有效力传感器。

## ROS 接口

左手命名空间 `/inspire/left`，右手 `/inspire/right`。以下名称均相对该命名空间；消息和服务定义在 [`rh5dg2_interfaces`](src/rh5dg2_interfaces)。

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `set_values` | `srv/SetValues` | 设置五种控制值，返回实际下发值及限幅标记 |
| `set_enabled` | `srv/SetEnabled` | 仅使能/禁用软件接收命令，不写设备寄存器 |
| `get_state` | `srv/GetState` | 读取缓存状态 |
| `stop` / `clear_stop` | `std_srvs/srv/Trigger` | 显式急停（1301=1）/解除急停（1301=0），均保持软件禁用 |
| `clear_errors` / `reconnect` | `std_srvs/srv/Trigger` | 清错、重连 |
| `command` | `msg/HandCommand` | 带类别、时间戳和命令 ID 的控制 |
| `angle_set` / `speed_set` / `force_set` / `current_limit_set` / `mode_set` | `msg/JointCommand` | 五种独立控制话题 |
| `command_result` | `msg/CommandResult` | 通过命令 ID 关联控制结果 |
| `state` | `msg/HandState` | 13 通道聚合反馈及有效标志 |
| `angle_actual` / `force_actual` / `current_actual` / `temperature` / `motor_status` / `error_code` | `msg/JointFeedback` | 独立反馈、单位、采集时间及通道掩码 |
| `touch` | `msg/TouchState` | 指尖触觉和掌部原始数据；best effort QoS |
| `joint_states` | `sensor_msgs/msg/JointState` | 可选的标定后角度，默认关闭 |

状态查询与话题共用缓存，不额外访问串口。使用反馈时检查对应 `valid`、采集时间和 `channel_present`；失效时保留最后值与时间戳。`get_state` 成功只说明关键反馈可用，不保证所有可选反馈有效或电机无错误。独立反馈使用 reliable / transient local / 深度 1，`state.header.stamp` 则是快照发布时间。

默认逐寄存器读取：角度 5 Hz，力/电流各 2 Hz，触觉及温度/状态/错误码各 1 Hz；聚合状态发布 20 Hz。采集率受串口和设备响应限制，各通道有采集时间差。反馈保持原始值，不随目标限幅。

## 运行边界

- 仅支持厂商串口协议，同一串口只能运行一个驱动。打不开设备时退出；超时或协议错误会断开、撤销使能并使缓存失效，不自动重连或重发。
- 默认 `enable_write: true`，首次反馈有效且错误码为零后接收命令，无需示例调用使能服务。可显式禁用写入；重新使能要求反馈有效且错误码为零。使能/禁用只改变软件是否接收命令，启动、退出及反馈异常均不自动写急停寄存器；设备原目标仍保留，软件禁用不表示运动停止。通信与电机错误仍会拒绝命令。
- `clear_stop_on_enable` 参数已移除。若旧版示例留下急停状态，先重启新驱动，再显式运行 `ros2 run rh5dg2_py hand_cli --namespace /inspire/left clear-stop`。该操作写 `1301=0` 并验证读回，保持软件禁用；再执行 `hand_cli --namespace /inspire/left enable` 恢复软件接收命令。
- 左手角度整组/单字/停止 ACK 分别为 32/2/2，右手为 1/1/1；非标准应答须精确读回验证，配置保存在两份 YAML 中。
- 多个单寄存器写入不是原子操作，失败时可能已有部分目标执行；丢失 ACK 也不能证明设备未动作。写入成功后的到位判断应使用新角度反馈。
- topic 命令需带新时间戳，默认最多 0.5 秒且不得早于本次使能。命令 ID 只用于结果关联，不去重。服务适合有应答的设置，不提供轨迹调度。
- `stop` 是通过通信请求固件停止；链路中断时不能保证立即停止。不提供硬实时控制或设备端 watchdog。
- `joint_states` 需要 `publish_joint_states`、`joint_mapping_confirmed` 同时开启，并核对 `joint_zero_raw` / `joint_direction`；换算为 `(raw-zero)*direction*pi/1800`。模型见 [description 说明](src/rh5dg2_description/README.md)。

## 验证和目录

```bash
./scripts/test.sh
```

测试覆盖协议、限幅、客户端、启动配置及双手虚拟串口集成；不会打开物理串口。详见 [测试说明](tests/README.md)，历史材料见 [验证记录](tests/VALIDATION.md) 和 [串口核查记录](docs/serial_readback_20261013.md)。

`src/` 下按接口（`rh5dg2_interfaces`）、驱动（`rh5dg2_driver`）、启动（`rh5dg2_bringup`）、客户端（`rh5dg2_py`）和参考模型（`rh5dg2_description`）组织。
