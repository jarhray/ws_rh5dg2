# rh5dg2_py

RH5DG2 的 topic 角度示例，以及可选的同步服务客户端和 CLI。默认命名空间 `/inspire/left`，右手使用 `/inspire/right`。先启动驱动，并在当前终端 `source install/setup.bash`。

## 修改数值运行示例

编辑 [`angle_demo.py`](rh5dg2_py/angle_demo.py) 顶部 `NAMESPACE` 和 `TARGET_ANGLES`。例如只改变下标 4 的食指根部：

```python
NAMESPACE = '/inspire/left'
TARGET_ANGLES = [-1, -1, -1, -1, 1700, -1, -1, -1, -1, -1, -1, -1, -1]
```

```bash
ros2 run rh5dg2_py angle_demo
# 也可覆盖左右手选择：
ros2 run rh5dg2_py angle_demo --namespace /inspire/right
```

数组固定 13 项，单位为 0.1 度原始寄存器值；`-1` 不写该通道。角度由驱动按[配置范围](../../README.md#通道和范围)截断。symlink install 下修改 Python 数值后重新运行即可。

示例只向 `<namespace>/angle_set` 发布一次 `JointCommand`。不创建服务客户端，不调用 `get_state`、使能/禁用、急停或解除急停，也不订阅状态或等待到位；退出只释放本地 ROS 资源。限幅由驱动执行。

单次发布前最多等待 5 秒建立 ROS 订阅连接，发布后最多等待 1 秒 DDS 接收确认，避免进程退出前消息丢失。“已发布”不表示驱动已执行或关节已到位。需要这些信息时，另行订阅 `command_result` 和 `angle_actual`。

默认启动配置 `enable_write: true`，首次有效反馈后直接接收 topic 命令。如果主动用 `enable_write:=false` 启动，或调用过禁用/急停/重连，需要由调用者显式恢复命令接收；示例不会修改这些状态。

## 可选的服务客户端

```python
from rh5dg2_py import HandClient, HandError

try:
    with HandClient('/inspire/left', timeout_sec=2.0) as hand:
        result = hand.set_angle([-1, -1, -1, -1, 1600, -1, -1, -1, -1, -1, -1, -1, -1])
        print(list(result.applied_values), list(result.clamped))
except HandError as exc:
    print(f'操作失败: {exc}')
```

`get_state()` 返回 `HandState`；`get_state_response()` 返回完整服务响应。关键反馈不可用或过期时抛异常，其他反馈仍需分别检查 `*_valid` 和 `*_stamp`。触觉在独立 `touch` 话题中提供。

| 方法 | 输入 |
| --- | --- |
| `set_angle` | 13 个整数，原始角度；驱动统一限幅，`-1` 跳过 |
| `set_speed` | 13 个整数，0–2500 |
| `set_force` | 13 个整数，0–1000，厂商力标度 |
| `set_current_limit` | 13 个整数，0–1500 mA |
| `set_mode` | 13 个 0/1；2/3、4/5 必须成对一致 |
| `set_values(kind, values)` | 上述五类的统一入口，`kind` 为 0–4 |

成功的设置响应含 `applied_values`（实际下发目标）和 `clamped`（逐通道限幅标记），不表示机械关节已到位。失败时这些数组无效。角度客户端只检查整数、长度和 int16 范围，限幅由驱动执行；其他设置越界会拒绝。

部分角度控制逐个写入非 `-1` 通道，其余通道原目标不变；全部指定时整组写入。多个单字写入中途故障可能已有部分目标执行，驱动断开并停止后续写入。`-1` 不是停止，也不能表示侧摆 -0.1°。

`set_enabled(True/False)` 仅控制软件是否接收命令，不写设备寄存器。`stop()` 显式写急停 `1301=1`；`clear_stop()` 显式写 `1301=0` 并核对读回，两者均使软件保持禁用。`clear_errors()` 清除设备错误；`reconnect()` 重建连接后保持禁用。客户端不自动解除急停、清错、重连或重试写入。

每个客户端使用独立 ROS context，不影响已有程序的全局 `rclpy` 状态。`close()` / 退出 `with` 只释放本地资源。`timeout_sec` 是每次调用发现服务和等待应答的总预算；超时抛 `HandTimeoutError`，驱动拒绝抛 `HandRejectedError`，两者均继承 `HandError`。超时不能证明设备未执行命令。

## CLI 和话题

```bash
ros2 run rh5dg2_py hand_cli --namespace /inspire/left state
ros2 run rh5dg2_py hand_cli --namespace /inspire/left enable
ros2 run rh5dg2_py hand_cli --namespace /inspire/left set-angle -1 -1 -1 -1 1700 -1 -1 -1 -1 -1 -1 -1 -1
ros2 run rh5dg2_py hand_cli --namespace /inspire/left disable
```

不带子命令时只读状态。另有 `stop`、`clear-stop`、`clear-errors`、`reconnect`，以及 `set-speed`、`set-force`、`set-current-limit`、`set-mode`、`raw KIND`，设置命令后跟 13 个值。旧版遗留急停需显式 `clear-stop`，再用 `enable` 恢复软件接收命令；topic 示例不会替调用者操作。

服务客户端中的 `service unavailable: .../get_state` 表示在超时时间内未发现 ROS 服务，不代表关节未到位或急停锁定。检查驱动终端是否仍在运行，两端命名空间和 `ROS_DOMAIN_ID` 是否一致。

`publish_command(kind, values, command_id=None)` 非阻塞发送并返回命令 ID，需订阅 `command_result` 确认结果。也可发布独立 `angle_set` / `speed_set` / `force_set` / `current_limit_set` / `mode_set` 话题，消息类型 `JointCommand`。所有 topic 控制都需要新时间戳；命令 ID 用于关联结果，不做去重。

六个独立反馈话题和完整接口见[工作区说明](../../README.md#ros-接口)。查看数值例如：

```bash
ros2 topic echo /inspire/left/angle_actual --once --qos-durability transient_local
```

完整消息包含有效性、采集时间、单位和通道掩码。仅显示 `values` 会隐藏这些信息。
