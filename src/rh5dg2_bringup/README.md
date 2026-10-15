# RH5DG2 启动配置

完成构建并 `source install/setup.bash` 后：

```bash
ros2 launch rh5dg2_bringup hand.launch.py side:=left port:=/dev/ttyUSB0
ros2 launch rh5dg2_bringup dual.launch.py left_port:=/dev/ttyUSB0 right_port:=/dev/ttyUSB1
```

单手选择 `left` 或 `right`，命名空间为 `/inspire/left`、`/inspire/right`。双手必须使用两个不同串口；launch 会检查实际路径，防止两个别名指向同一端口。驱动也会独占打开串口。

## 参数

单手 launch 暴露 `side`、`port`、`baudrate`、`hand_id`、`enable_write`、`publish_joint_states`、`joint_mapping_confirmed`。除 `side` 默认 `left` 外，参数默认留空，沿用该手 YAML；显式传值才覆盖 YAML。

双手使用 `left_port` / `right_port`、`left_hand_id` / `right_hand_id`；`baudrate` 和三个布尔参数是两只手共用的覆盖值。要分别设置，直接修改各自 YAML。

配置文件为 [`config/left.yaml`](config/left.yaml) 和 [`config/right.yaml`](config/right.yaml)，驱动启动时读取，修改后需重启。

| 配置 | 默认值或用途 |
| --- | --- |
| `port` | `/dev/rh5dg2_left` / `/dev/rh5dg2_right`，预留别名，不会自动创建 |
| `baudrate` / `hand_id` | 115200 / 1；串口 8N1 |
| `enable_write` | `true`，首次有效反馈后接收命令；设 `false` 可只读启动 |
| `serial_timeout_ms` | 40，单次串口事务超时 |
| `command_max_age_sec` | 0.5，topic 命令最大年龄 |
| `angle_rate` | 5 Hz |
| `force_rate` / `current_rate` | 2 Hz；可设 0 关闭 |
| `touch_rate` | 1 Hz；可设 0 关闭 |
| `slow_rate` | 温度、状态和错误码各 1 Hz |
| `state_rate` | 聚合状态发布 20 Hz，不代表采样率 |
| `publish_joint_states` / `joint_mapping_confirmed` | 均默认 `false`；开启前核对机械映射 |
| `joint_zero_raw` / `joint_direction` | 可选的 13 项零点/方向数组，用于弧度换算 |

串口固定逐寄存器读取，整组完成时标记采集时间。角度固定按[工作区文档范围](../../README.md#通道和范围)截断，不再配置 profile、读取模式或限幅策略。

## 两侧设备差异

| 配置 | 左手 | 右手 |
| --- | --- | --- |
| `angle_write_ack`，整组角度 | 32 | 1 |
| `single_angle_write_ack`，单通道角度 | 2 | 1 |
| `stop_write_ack`，停止标志 | 2 | 1 |

非标准 ACK 需要读回目标或标志并精确匹配。停止 ACK 仅作用于 `1301=0/1`，清错和其他设置仍要求 ACK=1。

两侧使能/禁用都只改变软件接收命令的开关，不写设备寄存器。`clear_stop_on_enable` 参数已移除；只有显式 `stop` / `clear_stop` 服务会写急停寄存器 `1301`。`clear_stop` 核对 ACK 和零值读回，随后仍需使能并发送新目标。默认启动在首次有效反馈后开放命令接收；显式重连后保持禁用，不重发旧目标；驱动退出也不发送急停。

设备路径不存在、无权限或被占用时节点报错退出；运行中超时或协议错误会断开并使反馈失效，通过 `reconnect` 服务显式恢复。
