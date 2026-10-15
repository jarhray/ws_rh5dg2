# 初版验证记录

本文保留历史行为与验收结果。当前使能/禁用已与急停分离，`clear_stop_on_enable` 已移除；当前示例只发布一次 angle_set topic，不读反馈或操作急停。当前接口以[工作区说明](../README.md)为准。

当前运行库已于 2026-10-13 删除模拟后端，只支持串口实机通信；最新验收见文末“仅保留实机后端”。下文提及 mock 的段落是历史版本记录，不代表当前仍提供此功能。

后续实机只读验证与串口兼容修正见 [2026-10-13 诊断记录](../docs/serial_readback_20261013.md)。下文保留 2026-10-12 初版离线验收范围。

日期：2026-10-12。环境：本机 ROS 2 Humble、系统 Python 3.10、C++17。机械手已断开；整个验收未打开物理串口或发送实机命令。

| 检查 | 结果 |
| --- | --- |
| 五个包 `colcon build --symlink-install` | 全部通过 |
| `colcon test-result --verbose` | 30 项，0 errors，0 failures，0 skipped |
| C++ 协议组 | 10 组通过，覆盖帧重同步、24 位触觉、短帧、ACK 校验、超时、独占与断连 |
| C++ 命令策略组 | profile、限位、哨兵、模式配对及左右关节命名检查通过 |
| Python 客户端 | 27 项通过 |
| launch 保护组 | 4 个用例通过，包含同串口别名冲突拒绝 |
| 双手 mock 的 ROS 集成 | 服务、topic、反馈时间戳、左右隔离、过期及跨使能旧命令拒绝全部通过 |
| 串口后端 + 内部 PTY 的 ROS 集成 | 默认零写、ACK、丢 ACK 停用且不重试、重连不重放全部通过 |
| 右手参考模型 | xacro 展开、URDF 树及 27 个网格引用验证通过 |

`colcon` 的 30 项由 27 个 Python 用例和 3 个 CTest 入口组成；C++ 与 launch 入口内还有上表列出的分组检查，不能把所有数字相加作为独立测试总数。

本机复现入口：`./scripts/test.sh`。最终通过的集成日志位于
[`log/integration/20261012T082121Z-4/`](../log/integration/20261012T082121Z-4/)，
完整测试输出保存在 [`log/validation-v1.log`](../log/validation-v1.log)。
沙箱禁止 UDP 套接字，Fast DDS 打印了网络传输警告；本机进程仍完成全部服务和话题验收。
这不验证跨机器 DDS 网络连通性。

实机待验收：转换器/手端协议与设备路径、手 ID 和波特率、拇指 profile、停止标志的停止及恢复行为、标定与真实反馈频率。CAN-FD、EtherCAT 后端尚未实现；左手参考模型已在下述增补中加入。

## 同日增补：左手镜像参考模型

右手源沿 XZ 平面派生左手 URDF、27 个 STL、溯源 manifest 与可复现生成脚本。原右手源和驱动控制逻辑未改动。

- description 重新构建通过；新增 2 个 CTest 入口通过：镜像模型验证（内含 5 个独立检查）及产物一致性。
- 23 组姿态下的全部 27 个 link 满足完整运动链镜像关系，含 13 个主动关节和 5 个 mimic。
- 480,894 个三角形的顶点、法向、绕序及关节运动后的 visual/collision 表面点通过检查；各 link 与整手的惯性、质心通过检查。
- 左手预览和反馈显示 launch 均实际运行通过；隔离 ROS 域内仅发布测试 `joint_states`，18 个动态 TF（含 mimic）与解析结果一致。未启动驱动、未发送 `command`、未连接实机，退出时所有子进程正常清理。

日志：[`mirror-tests.log`](../log/mirror-tests.log)、[`left_model_smoke/`](../log/left_model_smoke/)。
网格渲染对照图：[`left_right_reference.png`](../docs/left_right_reference.png)。
这些检查验证派生模型与右手源的数学一致性，不证明真实左手的外壳、安装和标定与镜像假设完全一致。

## 2026-10-13 增补：独立反馈与控制话题

为终端查看增加六个独立反馈话题，以及五个按用途命名的控制话题。保留原聚合消息、服务与 Python API；独立反馈共用采集缓存。

- 五个包重新构建成功；`colcon test-result` 为 32 项，0 errors、0 failures、0 skipped。
- 双手 mock 验收核对每侧六种反馈的数值、单位、采集时间、通道名称及力传感器掩码。
- 新控制入口均通过禁用门控、过期/未来/缺失时间戳、限位和模式配对检查；`angle_set` 验证使能时刻检查与左右隔离。
- 内部 PTY 核对五种命令各写入一次正确寄存器，数值未经意外修改；模拟丢 ACK 后六种反馈均失效，保留上次数据与时间戳；迟加入的订阅者可获得保留的无效消息，显式重连后反馈恢复且不重放命令。

构建日志：[`build-split-topics.log`](../log/build-split-topics.log)。
完整测试日志：[`test-split-topics.log`](../log/test-split-topics.log)。
集成日志：[`20261013T063100Z-4`](../log/integration/20261013T063100Z-4/)。
本轮只运行 mock 与内部虚拟串口，未打开物理设备；Fast DDS 在受限环境提示 UDP 传输不可用，本机进程通过共享内存完成验收。

## 2026-10-13 增补：可编辑角度示例

新增 `rh5dg2_py/angle_demo.py` 与 `ros2 run rh5dg2_py angle_demo` 入口。
Python 包重新构建成功；本轮重新运行 Python 的 40 项测试（含示例新增的 13 项），全部通过。
示例覆盖未确认 profile/无效反馈/错误码的使能前拒绝、只接受写入应答之后的新角度采样、
跳过通道、写入失败及中断后的禁用、旧缓存超时、清理失败时保留原错误。

双手 mock 集成实际运行 `run_angles()`，验证目标反馈、未指定通道及另一只手不变、
完成后命令门关闭；内部 PTY 串口回归也全部通过。`ros2 run ... --help` 入口检查通过。
之后仅将输出中的 NumPy 整数格式改为普通整数，并再次通过全部 13 项示例测试。
本轮未访问实机。

日志：[`build-angle-demo.log`](../log/build-angle-demo.log)、
[`test-angle-demo.log`](../log/test-angle-demo.log)、
[`20261013T072902Z-4`](../log/integration/20261013T072902Z-4/)。

## 2026-10-13 增补：仅保留实机后端

删除运行时 `MockDevice`、`mock_demo` 及三种反馈消息的 `mock` 字段，launch 和 YAML 默认改为串口。保留 `transport:=serial` 的调用方式，其他传输类型报错。
找不到串口或端口被占用时节点打印具体原因并以非零状态退出；运行中通信故障仍按原逻辑失效、禁用，等待显式重连。

- 五个包重新构建成功；43 项测试全部通过（38 项 Python 用例、5 个 CTest 入口）。
- 串口 ROS 集成使用内部创建的两个 PTY，通过实际串口后端覆盖双手接口、命令校验、独立反馈、24 位触觉、角度示例、丢 ACK 失效、清错及重连，不再启动模拟后端。
- 不存在的串口：验证节点非零退出，输出设备路径和 `No such file` 原因。
- 已占用串口：验证第二个节点非零退出，未发送寄存器写入。
- `transport:=mock`：验证启动失败，没有后备传输模式。
- 安装产物核对：Python 可执行入口仅为 `hand_cli`、`angle_demo`，三种消息均无 `mock` 字段。

本轮未访问物理串口。测试应答器仅保留在 `tests/` 中，不安装为库的后端。
日志：[`build-hardware-only.log`](../log/build-hardware-only.log)、
[`test-hardware-only.log`](../log/test-hardware-only.log)、
[`20261013T073632Z-4`](../log/integration/20261013T073632Z-4/)。

## 2026-10-13 增补：异常 ACK 与错指动作诊断

用户报告角度命令的非标准 ACK 及中指动作。获得停止驱动、只读诊断授权后，
实际执行了 39 次单寄存器读取，未发送运动、停止、清错或任何寄存器写入。
目标 1084 回读为 1700，其余目标为 -1；反馈 1138 为 990、1140 为 1650，错误码全零。
完整原始请求/回包见 [`control_readback_20261013.jsonl`](../log/read_only_diagnosis/control_readback_20261013.jsonl)。
这只能证明目标寄存器内容与用户请求一致，不能证明物理动作或关节映射正确。
旧驱动未记录非标准 ACK 的实际字节，无法从这次只读数据恢复它。

驱动将确定性的“Hand rejected”改为“Unexpected write ACK”，记录 ACK 原值、地址、字数及完整收发帧；
仍然断开、禁用且不重试。角度示例增加目标下标/寄存器显示，明确软件禁用不证明物理停止。
用户编辑的目标值保留，未变更通道映射或放宽 ACK 检查。

重新构建 driver/py 成功；本轮执行 38 项 Python 测试及 2 个 driver CTest 入口，全部通过。
核心协议回归验证 ACK=0 不被接受且诊断帧/地址/字数准确；完整串口 ROS 集成通过，
新增检查证明 `--control-readback` 只发指定的 39 个单字读取、零写入。
汇总中的 43 项包含其余包之前留下的测试结果，不表示本轮重新运行了所有包。

日志：[`build-ack-diagnostics.log`](../log/build-ack-diagnostics.log)、
[`test-ack-diagnostics.log`](../log/test-ack-diagnostics.log)、
[`20261013T075717Z-5`](../log/integration/20261013T075717Z-5/)。
运动异常根因尚未确认；这轮结果不算实机运动修复或验收通过。

## 2026-10-13 增补：厂商完整目标对照与当前左手 ACK 适配

用户已使用原厂 `inspire_control_single_device.launch.py device_name:=hand_left` 配合完整角度数组控制运动。
核对其实际安装配置仍为 RH5DG2_485、115200、ID 1。用原厂已编译协议对象构建对照程序，
对张开姿态、首两指弯曲姿态及此前的 -1 数组，原厂构帧与本工作区真实串口后端在内部 PTY 上的输出逐字节一致。
同时验证原厂解析器接受 ACK=0，且解析接口本身不核对请求地址/所需字数；发布层会为缺少的通道补零。
上述解析问题是代码可确认的缺陷，不足以单独解释用户那一次 `112` 的来源。

获用户停止原厂驱动并授权只读后，读回张开目标完全一致；逐字角度为
`[1648,1648,1651,0,1652,0,1850,1851,1851,1851,1702,1580,2040]`，错误码全零。
另对同一地址分别读 26 字节和 2 字节，捕获整组仅首项 1648、其余全零；单寄存器读取得到完整各项。
这一证据证明整组读取的问题也存在于设备/链路返回内容，不能只改 ROS 消息发布就解决。

随后获得用户明确授权，仅向真实左手重发一次其已验证、仍保存在设备中的完整张开目标。
帧为 `eb 90 01 1d 12 38 04 08 07 08 07 08 07 00 00 08 07 00 00 3a 07 3a 07 3a 07 3a 07 a4 06 40 06 f8 07 9b`，
回包 `90 eb 01 04 12 38 04 20 73`，校验正确且 ACK=32。旧标准 ACK=1 检查因此报错。
未再次写入；后续只读确认目标未改变，实际角度仍如上。停止/清错/速度/力等 ACK 未做实机验证。

本轮修正：

- `left.yaml` 保存 `angle_write_ack: 32`，右手仍为 1。此参数仅覆盖 1080 的完整 13 通道角度写入；非标准应答之后强制单寄存器读回全部目标，精确相等才成功，不重试。
- `-1` 改为驱动层保留原有目标，先读回目标再合并限位检查；不取当前位置代替、不自动夹紧，待保留的目标无效则拒绝，全跳过不写入。
- 拒绝角度整组零填充样本；正常零电流/错误码仍允许。寄存器顺序和用户编辑的 demo 目标未改变。

driver/py/bringup 重新构建通过；本轮 38 项 Python 用例、driver 的两个 CTest（核心协议内含 12 组）
和 bringup 的一个 CTest（6 组）全部通过。完整双手 PTY ROS 验收也通过，覆盖左手 ACK=32+读回、右手标准 ACK、
原目标与实际角度不一致时保留原目标、-1 无效基线拒绝、丢 ACK 不重试及所有既有接口。
43 项汇总含 description 包之前的测试结果，该包未在本轮重跑。

最后使用重新构建的真实 `hand.launch.py` 启动当前左手，强制 `enable_write=false`，只调用 GetState 三次，
验证 YAML 加载 ACK=32、反馈新鲜、错误码零、未使能，并正常退出该临时驱动。此步骤没有发送运动或停止。
本轮总共向真实硬件发送一条经授权的角度写入，不是全套运动验收；-1 与此前错指的因果关系、实际到位及停止/恢复仍待验证。

证据与日志：

- [`vendor_wire_comparison_20261013.log`](../log/vendor_wire_comparison_20261013.log)
- [`vendor_reference_ack_20261013.log`](../log/vendor_reference_ack_20261013.log)
- [`vendor_control_readback_20261013.jsonl`](../log/read_only_diagnosis/vendor_control_readback_20261013.jsonl)
- [`vendor_block_vs_single_20261013.jsonl`](../log/read_only_diagnosis/vendor_block_vs_single_20261013.jsonl)
- [`reference_after_ack32_20261013.jsonl`](../log/read_only_diagnosis/reference_after_ack32_20261013.jsonl)
- [`ros_vendor_compatibility_state.json`](../log/read_only_diagnosis/ros_vendor_compatibility_state.json)
- [`build-vendor-compatibility.log`](../log/build-vendor-compatibility.log)、[`test-vendor-compatibility.log`](../log/test-vendor-compatibility.log)
- [`20261013T114343Z-5`](../log/integration/20261013T114343Z-5/)

## 2026-10-13 增补：demo 退出时停止 ACK=2

用户运行适配后的 demo，将下标 4 的目标设置为 1200，反馈为 1204，并确认动作正确。
随后退出时写 `1301=1`，发送帧 `eb 90 01 05 12 15 05 01 00 33`，
返回 `90 eb 01 04 12 15 05 02 33`。旧驱动固定要求停止 ACK=1，因而断开；
后续 demo 的 get_state 失败来自该保留的故障，未发送新的角度目标。

本轮实机仅做三次单寄存器读取，得到停止标志 1301=1、食指目标 1084=1200、
实际角度 1140=1198。未发送运动、停止、清错或复位写入。

修正：左手配置 `stop_write_ack: 2`，只覆盖单字写入 `1301=1`，
并要求随后读取该寄存器等于 1；读回不符、超时或其他 ACK 仍断开、不重试。
该配置不影响清错、其他设置、1301=0 或右手默认 ACK=1。
使能仍只打开软件命令门，不自动清除停止标志。服务消息明确区分固件回复与物理停止。
旧进程需重新启动加载参数和代码；只调用 reconnect 不会载入更新。

driver/bringup 构建通过。driver 两个 CTest（核心协议含 13 组）、38 项 Python 用例、
bringup 的一个 CTest（6 组）全部通过。协议测试复现上述停止帧与 ACK，覆盖匹配读回、
读回不符/丢失、未知 ACK、作用范围隔离及后续通信。完整双手 PTY ROS 验收通过：
左手角度 ACK=32、停止 ACK=2，右手均为 1；连续两次 run_angles 使用不同目标，
两次之间不重连，退出后连接保持、新鲜反馈可读、命令门禁用、跳过通道及右手不受影响。
43 项汇总含 description 包之前的两项结果，本轮未重跑该包。

上述离线验收不模拟真实停止标志对电机的作用，实机连续运动和停止/恢复行为仍待核对。
用户当前填写的 demo 目标数值未在本轮改动。

证据与日志：

- [`stop_ack2_readback_20261013.jsonl`](../log/read_only_diagnosis/stop_ack2_readback_20261013.jsonl)
- [`build-stop-ack.log`](../log/build-stop-ack.log)、[`test-stop-ack.log`](../log/test-stop-ack.log)
- [`20261013T115701Z-4`](../log/integration/20261013T115701Z-4/serial_test.log)

## 2026-10-13 增补：跳过通道越界与固件停止锁存

用户在食指目标 1800、其余 -1 时收到 `Cannot resolve skipped angle targets ... Channel 6 outside [1050,1850]`。
停止驱动后只读确认，设备保存目标为
`[1650,1650,1650,0,1650,0,1900,1900,1900,1900,1750,1600,2040]`，
实际角度为 `[1648,1648,1651,0,1198,-1,1850,1851,1851,1850,1702,1580,2040]`，错误码全零，1301=1。
原来的全数组补齐算法会重新验证并重发未修改的目标，因而被第 6～9 通道的 1900 阻塞；
拇指侧摆 1750 也超出手册命令限位。此次不扩大限位，也不将测量角度或夹紧值用于替换用户跳过的目标。

本轮实际写入均逐次获得用户明确授权，诊断程序先通过内部 PTY 检查，且检查原状态后才写入：

1. 单次 `1084=1800`，帧 `eb 90 01 05 12 3c 04 08 07 67`，
   实际回复 `90 eb 01 04 12 3c 04 02 59`（ACK=2）。仅该目标改变，实际角度仍 1198，用户确认没有移动。
2. 在独立电源接通、位置模式 0、速度 2000、错误码零的条件下，
   用原厂已验证的其他手指张开目标加食指 1500 做完整数组对照。ACK=32 且所有目标读回一致，
   连续 3 秒实际角度仍 1198。程序按约定中止，**没有发送第二个完整目标或任何停止写入**。
3. 用户另行授权恢复测试，先写一次 `1301=0`。帧 `eb 90 01 05 12 15 05 00 00 32`，
   返回 `90 eb 01 04 12 15 05 02 33`（ACK=2），随后读回 0。清零后的观察阶段实际角度没有自行变化。
   然后只写食指 1500，实际反馈从 1198 经 1458 到 1498；只写食指 1200，反馈经 1245 到 1205。
   最终目标数组仅食指变为 1200，错误码仍全零，停止标志为 0。没有再次发送停止、模式、速度、力或复位命令。

本轮共 **5 条实机寄存器写入**：单字角度 3 条、整组角度 1 条、解除停止 1 条，均无自动重试。
这证明当前左手支持单寄存器角度控制，以及停止标志需要清除后再发新角度；
未完成全通道机械行程或运动中的急停性能验收，也未确认脚本目标 1800 能达到相同实际反馈。

最终代码：

- 部分角度数组只逐个写非 -1 的寄存器；完整数组仍一次整组写入。所有指定值先校验，任何指定值越界整条请求零写入。
- 非标准单字角度应答需读取所写目标核对。左手 `single_angle_write_ack=2`；未修改目标为 1900/1750/-1 时不会阻塞、不会被重发。
- 多个单字写入不保证原子性，失败后不发送剩余通道，报错包含失败通道与已完成数量，断开且不重试。
- `stop_write_ack=2` 只覆盖 1301=0/1，核对所写标志；清零始终需要读回确认，即使使用标准 ACK=1。
- 当前左手 `clear_stop_on_enable=true`：使能请求检查停止标志，需要时清零并读回成功才打开门。
  读回不符、异常标志或通信错误都拒绝使能。默认启动、显式 reconnect 保持禁用，不重放目标。
  显式选择 `enable_write=true` 同样属于使能请求。右手尚未验证，恢复选项保持 false，三个 ACK 保持 1。

driver/bringup/py 构建及安装通过。38 项 Python 用例、driver 的两个 CTest（协议内含 14 组）、
bringup 一个 CTest（6 组）全部通过；43 项汇总包含 description 包的历史两项结果。
完整双手 ROS PTY 验收通过，包括：未修改目标越界时单指控制、所有指定目标先校验、失败后不写剩余通道、
停止状态下仅保存目标、解除后需要新命令才能运动、连续两次 demo 的停止/恢复/到位、
停止清零读回不符时拒绝使能并断开且不重试。没有将测试用应答器加入运行库。

证据与日志：

- [`skipped_targets_after_stop_20261013.jsonl`](../log/read_only_diagnosis/skipped_targets_after_stop_20261013.jsonl)
- [`single_angle_no_motion_20261013.jsonl`](../log/read_only_diagnosis/single_angle_no_motion_20261013.jsonl)
- [`single-angle-physical-20261013.log`](../log/single-angle-physical-20261013.log)
- [`full-motion-physical-20261013.log`](../log/full-motion-physical-20261013.log)
- [`stop-clear-and-angle-physical-20261013.log`](../log/stop-clear-and-angle-physical-20261013.log)
- [`build-sparse-resume.log`](../log/build-sparse-resume.log)、[`build-sparse-resume-incremental.log`](../log/build-sparse-resume-incremental.log)
- [`test-sparse-resume.log`](../log/test-sparse-resume.log)、[`20261013T135731Z-4`](../log/integration/20261013T135731Z-4/serial_test.log)

## 2026-10-14：统一角度限幅与代码整理（重构前）

此节记录重构前行为；下节为当前实现，上文“越界整条拒绝”是更早版本的历史行为。

- 驱动新增 `angle_limit_mode`，默认 `clamp`，也支持 `reject`。`angle_min_raw` / `angle_max_raw` 可同时覆盖 13 通道范围；非法配置在串口打开前拒绝。
- 命令策略在启动时建立，服务和两个角度话题共用。部分写入流程独立封装，保留 `-1` 跳过、完整数组整组写入、异常断连和不重试规则。
- `SetValues` 和 `CommandResult` 新增实际下发值与逐通道限幅标记。Python 示例按实际下发值等待反馈，保留退出停止和故障处理。
- 清理 README 的开发过程叙述和过时补齐说明、冗余状态变量、串口专用分支和测试 runner 的未使用参数；补齐构建环境隔离。

验证：全工作区构建通过；`./scripts/test.sh` 汇总 **44 tests, 0 errors, 0 failures, 0 skipped**（39 项 Python、driver 两项 CTest、description 两项 CTest、bringup 一项 CTest）。C++ 限幅测试覆盖全部通道上下界、int16 极值、负侧摆、跳过哨兵、自定义/固定范围、非法配置和严格模式。Python 新增截断后到位判断用例。

双手 PTY ROS 验收通过，实际核对截断后的串口写入、跳过通道零写入、服务及话题响应字段、自定义下界、严格拒绝、错误配置启动失败，以及既有停止恢复/丢 ACK/双手隔离行为。日志见 [`20261014T060311Z-2`](../log/integration/20261014T060311Z-2/serial_test.log)。受限环境报告 UDP 接口不可用，测试通过同机通信完成。

本轮没有打开物理串口或执行实机运动。尚未收到完整实测端点，现有默认范围未扩大或缩小；文档明确标注其为软件默认值，不作为完整机械行程标定结论。更新接口后必须重建并重启驱动及客户端。

## 2026-10-14：按文档范围简化重构

当前结构为具体 `SerialDevice`、固定寄存器/限幅函数、一个 ROS 节点和薄 Python 客户端。删除虚接口/工厂、profile、可配置限位策略和 diagnostics；保留现有服务/话题、逐字串口、ACK 读回、停止恢复、有效性与断连规则。范围按用户要求固定为文档值，拇指第一弯曲 1220–1600。

全工作区构建通过；`./scripts/test.sh` 汇总 **47 tests, 0 errors, 0 failures, 0 skipped**。14 组协议场景、固定限幅、42 项 Python 用例、6 项 launch 用例及模型测试通过。双手 PTY 集成通过，覆盖串口实际下发值、五种命令、跳过通道、状态失效、停止恢复、连续两次示例、丢 ACK 不重试、重连及双手隔离。日志：[`20261014T062221Z-2`](../log/integration/20261014T062221Z-2/serial_test.log)。测试未连接实机。

节点 609→444 行，Python 使用层 536→443 行，README 237→117 行。更新方式和已删除参数见工作区 README；此处历史验收描述不作为当前接口说明。
