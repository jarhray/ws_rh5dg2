# RH5DG2 左右手参考模型

右手来源：相邻原厂目录 `rh5dg2/RH5DG2_URDF/src/urdf/RH5DG2_R_2026_01_12.xacro` 和其引用的 27 个 `meshes/*.STL`。原厂包标注 BSD，本包保留该声明，未推定其他授权。右手只更换网格包引用和 xacro 文件名，机械参数保持原样。

左手由右手沿根坐标系的 **XZ 平面镜像（Y 取反）** 得到。手指在原模型主要沿 Z 延伸、根部沿 Y 排列，所以该平面交换拇指所在的横向侧。它包含实际镜像后的网格、关节变换和惯性，可以用于左右手预览、运动学开发和核对反馈；这是派生参考模型，不是厂商提供的左手 CAD 或实机标定结果。

## 使用

工作区构建并 `source install/setup.bash` 后：

```bash
# 左手预览；滑块只发到 /rh5dg2_preview/left/joint_states
ros2 launch rh5dg2_description display.launch.py side:=left

# 右手预览；仍为原来的默认选项
ros2 launch rh5dg2_description display.launch.py side:=right

# 左手反馈显示：订阅 /inspire/left/joint_states，不启动驱动或滑块
ros2 launch rh5dg2_description real_feedback.launch.py side:=left
```

预览的左右命名空间及 TF 话题各自独立，不会给实机发送控制命令。可用 `rviz:=false` 关闭 RViz，预览还支持 `gui:=false`；关闭 GUI 后只会发布固定关节变换，不自动补发可动关节状态。

反馈显示的 `side` 决定对应模型、RViz 固定坐标系和 `/inspire/<side>/joint_states` 订阅。驱动实机 `joint_states` 仍要求`joint_mapping_confirmed:=true`。镜像模型采用**同样关节角 q 产生镜像姿态**的约定，不代表固件原始角度值要全部取负。左右 raw→q 的零点、方向仍需各自核对。

## 镜像方式与复现

源文件：`urdf/rh5dg2_right.xacro`。生成文件：`urdf/rh5dg2_left.xacro`、`meshes/left_mirrored/left_*.STL`。生成文件已经随工作区保存，启动时不需要安装网格处理库或重新生成。

```bash
# 在工作区根目录运行，仅依赖 Python 标准库
/usr/bin/python3 src/rh5dg2_description/tools/generate_left.py
# 验证已保存产物与当前右手源一致，不写入
/usr/bin/python3 src/rh5dg2_description/tools/generate_left.py --check
```

令 `S=diag(1,-1,1)`，所有局部坐标系统一处理：

| 数据 | 左手变换 |
| --- | --- |
| origin / 质心位置 | `p_left=S*p_right` |
| origin 旋转 | `R_left=S*R_right*S`；RPY 为 `(-roll,pitch,-yaw)` |
| 转动关节轴 | `axis_left=(-ax,ay,-az)`，轴属于旋转向量 |
| 惯性张量 | `I_left=S*I_right*S`，`ixy`、`iyz` 取反；质量不变 |
| STL 网格 | 顶点和法向量 Y 取反，每个三角形交换两个顶点保持正面朝外 |
| 名称 / 引用 | `right_` 换为 `left_` |
| 关节角、限位、mimic 系数/偏置 | 保持原数值 |

每个 link 的坐标系仍为右手坐标系，符合 [ROS REP 103](https://github.com/openrobotics/reps/blob/main/_posts/rep-0103.md)。没有使用单个负 scale 来代替整条运动链的镜像。

独立测试在随机的 13 路主动关节姿态下计算整棵运动学树，包括 5 个 mimic 关节，并验证左右各 link 姿态与表面点的镜像关系，同时检查网格朝向、惯性和资产引用。随包构建后的测试入口：

```bash
colcon test --packages-select rh5dg2_description
colcon test-result --verbose
```

## 尚需实机核对的部分

左右外壳、接口位置和内部质量分布按严格镜像假设派生，不能据此证明真实左手完全对称。原右手模型自身的零位、轴和范围也尚未标定。特别是拇指 PIP 跟随 MCP、DIP 独立的定义，与旧厂商 Python 控制映射存在冲突；镜像保留原模型关系，不改变原始寄存器通道映射。

安装到机械臂时，法兰到机械臂的固定变换应按实际装配另行配置，不应再给模型重复镜像。
