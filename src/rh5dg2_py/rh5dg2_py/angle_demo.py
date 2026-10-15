"""修改下方 TARGET_ANGLES，通过 angle_set topic 发布一次角度目标。"""

import argparse
import sys
import time

from rh5dg2_py import CommandKind, validate_values

# 修改命名空间和目标即可运行。
NAMESPACE = "/inspire/left"  # 右手改为 '/inspire/right'

# 单位：原始寄存器 0.1 度（1700 = 170.0 度），不是 URDF 弧度。
# -1 跳过该通道，保留设备原目标；越界角度由驱动按配置限幅。
TARGET_ANGLES = [
    1650,  #  0 小指根部
    1650,  #  1 无名指根部
    1650,  #  2 中指根部
    0,  #  3 中指侧摆
    1650,  #  4 食指根部 <-- 修改这里即可试一个关节
    0,  #  5 食指侧摆
    1850,  #  6 小指第一弯曲
    1850,  #  7 无名指第一弯曲
    1850,  #  8 中指第一弯曲
    1850,  #  9 食指第一弯曲
    1700,  # 10 拇指侧摆
    1600,  # 11 拇指第一弯曲
    2040,  # 12 拇指第二弯曲
]

# TARGET_ANGLES = [
#     1000,  #  0 小指根部
#     1000,  #  1 无名指根部
#     1000,  #  2 中指根部
#     0,  #  3 中指侧摆
#     1000,  #  4 食指根部 <-- 修改这里即可试一个关节
#     0,  #  5 食指侧摆
#     1150,  #  6 小指第一弯曲
#     1150,  #  7 无名指第一弯曲
#     1150,  #  8 中指第一弯曲
#     1150,  #  9 食指第一弯曲
#     -1,  # 10 拇指侧摆
#     -1,  # 11 拇指第一弯曲
#     1500,  # 12 拇指第二弯曲
# ]


def main(argv=None):
    import rclpy
    from rclpy.duration import Duration
    from rh5dg2_interfaces.msg import JointCommand

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default=NAMESPACE)
    args = parser.parse_args(argv)
    try:
        values = validate_values(CommandKind.ANGLE, TARGET_ANGLES)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init(args=[])
    node = rclpy.create_node("angle_demo", namespace=args.namespace)
    try:
        publisher = node.create_publisher(JointCommand, "angle_set", 1)
        # 只等待 ROS 连接，不读取状态、不调用服务、不等待关节到位。
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("angle_set 没有订阅者，请先启动驱动")
            rclpy.spin_once(node, timeout_sec=0.05)
        message = JointCommand()
        stamp = node.get_clock().now()
        message.header.stamp = stamp.to_msg()
        message.command_id = stamp.nanoseconds
        message.values = values
        publisher.publish(message)
        # 等待 DDS 接收确认再销毁 publisher；不代表驱动执行或机械到位。
        if not publisher.wait_for_all_acked(Duration(seconds=1.0)):
            raise RuntimeError("目标已发布，但 DDS 送达确认超时；未重发")
        print(f"已发布到 {publisher.topic_name}：{values}（-1 跳过）", flush=True)
        return 0
    except RuntimeError as exc:
        print(f"angle_demo：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
