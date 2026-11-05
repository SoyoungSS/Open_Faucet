# 右机械臂操作单把双自由度水龙头

新增：[头部较高固定相机 RGB-D 与 FoundationPose 快照接入说明](docs/foundationpose.md)。
可导出相机数据、刚体网格，并以外部估计位姿初始化规划；在线视觉闭环尚未实现。
使用 `bash run.sh --visual-pipeline --quit-on-complete` 可在同一次仿真中自动采集头部 RGB-D、
运行已安装的 FoundationPose、校验并规划执行；当前分割仍使用仿真真值 mask。

使用本目录的 `dual_arm` 双臂机器人和 `single_lever_faucet` 水龙头，在 **conda 环境 `dexonomy-isaac`** 中运行 Python / Isaac Sim 仿真。仅右臂执行任务，左臂保持初始位置。默认模式不使用 RL、IL、神经网络或训练数据；可选 FoundationPose 接入需要独立推理环境和预训练权重。

默认流程：**初始位置 → 把手前方 → 沿把手长度方向接近 → 上下闭爪抓取 → 抬升开阀 → 左右调温 → 调整开度 → 居中关闭 → 松爪 → 撤离 → 返回初始位置**。机器人位于龙头出水嘴朝向的前方，右肩对齐龙头；初始抓取时手指开合沿世界 Z 方向。

**当前代码已改为无固定抓取约束的接触/摩擦交互。** 机器人与龙头之间不创建固定关节或其他附着约束，龙头全程没有位置刚度和逐帧位置命令。此前的 `examples/validated_result.json` 属于旧版理想抓取演示，不能作为当前纯摩擦版本的验证结果；新实验记录见第 10 节。

## 仿真演示视频

约 **51 秒 · 1080p · 30 fps**，包含全景、夹持特写、俯视调温和侧视开关。

[![点击观看完整仿真视频](media/faucet_multiview_preview.gif)](media/faucet_multiview.mp4)

[▶ 打开完整视频](media/faucet_multiview.mp4) · [下载 MP4](https://github.com/4taotao8/Open_Faucet/raw/refs/heads/main/media/faucet_multiview.mp4)

## 获取项目

```bash
git clone https://github.com/4taotao8/Open_Faucet.git
cd Open_Faucet
```

以下命令均在项目根目录执行。先准备好已有的 `dexonomy-isaac` 环境；项目不会自动下载或安装依赖。

![上下夹持把手的实际仿真近景](examples/grasp_closed_detail.png)

## 1. 已验证环境与依赖

本机已存在以下软件，项目开发过程中没有安装新依赖：

| 软件/包 | 本机版本 | 用途 |
| --- | --- | --- |
| Python | 3.11.15 | 启动脚本和任务逻辑 |
| Isaac Sim / `isaacsim` | 5.0.0.0 | 场景、PhysX 物理、关节驱动和可视化 |
| NumPy | 1.26.0 | 矩阵、关节角与轨迹数组 |
| SciPy | 1.15.3 | 旋转变换、有界最小二乘 IK、Hermite 轨迹插值 |
| OpenUSD / `pxr` | 环境已有，usd-core 25.2 | 读取模型、关节和场景层；仿真运行时使用 Isaac 的 USD 运行库 |
| `omni`、`carb`、`PhysxSchema`、`omni.ui` | Isaac Sim 自带 | 运行时、物理配置、状态面板与截图 |
| NVIDIA 驱动 / RTX GPU | 本机 580.178.04 / RTX 4090 | Isaac Sim 渲染和运行 |

其余使用 `argparse`、`json`、`csv`、`pathlib`、`unittest` 等 Python 标准库。**不需要另装 Isaac Lab、ROS、MoveIt、cuRobo、PyTorch、Open3D 或规划服务。** `requirements.txt` 仅记录本机直接依赖，不建议对已有 Isaac 环境直接执行整份重新安装。

环境检查：

```bash
conda activate dexonomy-isaac
python scripts/check_environment.py
```

脚本只检查，不安装软件、不访问包仓库。若从受限的执行环境检查 GPU，可能需要允许显卡访问；本机已在获得显卡访问权限后实际启动并验证过 Isaac Sim。

## 2. 启动可视化仿真

```bash
conda activate dexonomy-isaac
python scripts/run_sim.py --realtime
```

也可以不用手动激活环境：

```bash
bash run.sh --realtime
```

`run.sh` 自动切换到项目目录，使用现有 `dexonomy-isaac` 环境。默认在流程完成后保留窗口，便于观察末态；关闭窗口结束。窗口中的 **Faucet trajectory demo** 面板显示实际左右角、实际抬升角、理想流量/温度和抓取状态。面板阶段使用英文标识，终端阶段说明、代码注释和本说明使用中文。

其他启动方式：

```bash
# 可视化执行完成后自动退出。
bash run.sh --realtime --quit-on-complete

# 无窗口物理验收，完成后自动退出。
bash run.sh --headless --quit-on-complete

# 保存每个阶段的实际视口截图和抓取局部图。
bash run.sh --headless --quit-on-complete --capture-frames --output outputs/my_demo

# 使用自定义任务配置。
bash run.sh --config configs/my_task.json --realtime
```

`--realtime` 限制每步执行速度，方便观看；不能将较慢的物理求解加速到实时。不加该选项时尽快执行，物理时间步不变。精细分段碰撞和 1/240 秒时间步增加了 CPU 求解开销：本机完整测试约 45 秒仿真时间，墙钟约 3 分 20 秒。`--headless` 仍需要支持 Isaac Sim 的 NVIDIA 显卡。所有场景几何和材质来源均在本地，不加载在线房间或远程机器人资产。

## 3. 修改左右角度与上下角度

复制配置再改动，保留一份默认可复现配置：

```bash
cp configs/default.json configs/my_task.json
```

修改 `operations` 数组。每项是一个**绝对阀角目标**，不是相对增量。例如：

```json
"operations": [
  {"name": "抬升到15度", "yaw_deg": 0, "lift_deg": 15, "duration": 3},
  {"name": "转向冷水侧10度", "yaw_deg": -10, "lift_deg": 15, "duration": 3},
  {"name": "转向热水侧20度并增大开度", "yaw_deg": 20, "lift_deg": 28, "duration": 4},
  {"name": "保持热水侧20度并减小开度", "yaw_deg": 20, "lift_deg": 8, "duration": 3},
  {"name": "关闭并回中", "yaw_deg": 0, "lift_deg": 0, "duration": 4}
]
```

- `yaw_deg`：左右调温，范围 **−45°～+45°**；负角为冷水侧，正角为热水侧。
- `lift_deg`：上下开关/开度，范围 **0°～35°**；0° 关闭。
- `duration`：该段期望时长，单位秒；若超过机器人速度/加速度设置，会自动延长。
- 同时改变两项会沿串联关节的复合运动路径操作。需要分开观察时，一次只改变其中一项。
- 最后一个操作是什么状态，就在什么状态松爪；不会偷偷追加关阀。需要关闭时，在数组最后明确写入 `lift_deg: 0`。

超出角度范围、出现 NaN/Inf、时间为负或目标不可达时直接报错，不静默截断目标。原始水龙头关节必须保持默认零位开始；本程序会在初始化时设置零位。

## 4. 场景和抓取参数

主要配置位于 `configs/default.json`。位置单位为米，配置角度为度；内部运动学、执行器及 CSV 中七轴角度统一用弧度。世界坐标采用 **Z 向上**。

| 参数 | 含义 |
| --- | --- |
| `robot_usd` / `faucet_usd` | 相对于项目根目录的资产入口 |
| `robot_position` | 机器人基座的世界位置；默认 `[0.17, 0.55, 0.95]`，使右肩对齐把手 |
| `robot_yaw_deg` | 基座绕世界 Z 的朝向，默认 −90°，朝向龙头 |
| `faucet_position` | 水龙头底部世界位置；默认 `[0, 0, 0.48]`；出水嘴朝向 +Y |
| `faucet_yaw_deg` | 水龙头在世界中绕 Z 轴的摆放角度 |
| `home_right_deg` | 右臂七轴初始角，顺序为 `arm_r_joint_1`～`7` |
| `tcp_in_gripper` | TCP 相对 `arm_r_link_gripper` 的局部位置 |
| `grasp_in_handle` | 抓取点相对水龙头 `handle` 刚体的位置 |
| `gripper_rpy_in_handle_deg` | 夹爪相对把手的 XYZ 欧拉角，默认 `[-90, 0, 0]`；夹爪 Y 开合方向对齐把手 −Z，手指伸出方向 −Z 对齐把手 −Y |
| `finger_open_deg` / `finger_closed_deg` | 原始单活动手指关节的张开/闭合目标 |
| `grasp_mode` | 当前仅支持 `friction`，禁止自动回退到固定约束 |
| `contact_static_friction` / `contact_dynamic_friction` | 接触静/动摩擦系数，默认 1.0 / 0.8；实验参数，不代表已校准实物 |
| `finger_stiffness` / `finger_damping` / `finger_max_effort` | 活动手指驱动刚度、阻尼和力矩上限；默认 4、1、0.3 N·m |
| `faucet_joint_damping` | 龙头两个关节的被动阻尼，默认 0.001；位置刚度始终为零 |
| `pregrasp_distance` | 沿夹爪局部 +Z，从抓取点退到预抓取点的距离 |
| `physics_dt` | 物理时间步，默认 1/240 秒，减小夹持接触的离散误差 |
| `knot_dt` | IK 轨迹节点的间隔，默认 0.1 秒 |
| `max_joint_velocity` / `max_joint_acceleration` | 规划速度/加速度上限，单位 rad/s、rad/s² |
| `transit_duration` / `approach_duration` | 初始转移与直线接近时长 |
| `grasp_duration` / `retreat_duration` | 闭爪/松爪与撤离时长 |
| `operations` | 按顺序执行的调温、开关角度目标 |

改变龙头位置后，抓取目标会由龙头实际结构重新计算，不是重放一套固定机器人关节角。但任意位置都不保证可达，也不保证能绕开新增障碍。默认场景与初始姿态已经实际测试；大幅修改摆放、TCP 或增加障碍后应重新验证。

抓取初始姿态下，夹爪长度方向与把手平行，开合方向沿世界 Z。随后夹爪随把手的转动保持相对姿态，因此抬升过程中开合轴也随把手倾斜。`grasp_in_handle` 的高度按原模型弯曲指尖的最低内表面确定，并非简单取把手包围盒中心；将其任意改回中心容易在接近阶段先顶到把手边缘。

## 5. 规划与物理执行原理

1. **读取原资产**：解析 USD 关节的 `body0/body1`、`localPos/localRot`、轴和限位，直接建立右臂 FK/IK。不会用另一个机器人模型替代本体。关节索引按名字解析，不假定数组顺序。
2. **把手位姿作为输入**：水龙头主体世界位姿和两个关节局部坐标系构成串联链。给定左右角与抬升角，计算把手世界位姿，再乘以局部抓取变换得到世界 TCP 目标。抬升轴随调温载体一起转动。
3. **轨迹规划**：起始转移和返回在关节空间插值；接近/撤离在 TCP 空间直线插值并保持姿态；左右/上下操作沿阀轴几何轨迹生成目标，逐节点做连续 IK。用五次时间标定平滑起止，再做单调 Hermite 关节插值并显式置零端点速度，避免限位附近过冲，根据最大速度/加速度放慢轨迹。最终插值为速度连续曲线，不是专用的跃度约束规划器。
4. **位置驱动跟踪**：每个物理步发送右臂七轴和手指位置目标，PhysX 计算实际运动。`set_joint_positions` 仅用于初始化，不用于逐帧播放或操作把手。
5. **接触与摩擦传力**：到达把手并闭爪后，检查实际 TCP 误差（小于 12 mm、8°），并要求上下两指都已有真实接触，才登记逻辑抓取状态。**登记不会改变 USD，不创建任何跨对象关节。** 活动手指由受限力矩夹紧把手，法向接触力与 PhysX 摩擦传递操作力。初始状态设置后，龙头全程没有位置命令，接近、闭爪、操作和释放时的位置刚度均为零。规划中的理想阀角只用于生成机器人目标和计算实测误差，不送入龙头控制器。
6. **验收与释放**：每个操作终点等待 0.5 秒收敛，读取 PhysX 实际阀角，要求与目标偏差小于 2°。同时记录把手相对夹爪的位姿变化（打滑/接触滚动）、两指法向力和摩擦冲量。松爪直接张开手指，不锁定龙头末态；把手可由重力和被动阻尼自行运动。运行偏离 TCP 轨迹超过 50 mm 或 20°时停止并报告失败。

### 需要明确的简化

- 当前没有理想无滑移约束，也不屏蔽夹爪与把手碰撞。右夹爪、活动手指及把手按原网格沿长度方向每 2 mm 分段裁剪，再逐段取凸包，保留弯曲指尖和夹爪开口。新碰撞网格位于各刚体的 `contact_hulls/`；旧的整块凸包停用，视觉网格不变。源资产本身的 articulation 自碰撞设置保持原状。真实传力同时包含法向接触和切向摩擦，不能把法向推力称为摩擦力。
- 每个物理步读取接触点、接触距离和法向冲量，统计最大穿透；超过 **0.8 mm** 即失败。该值是物理离散求解的容限，不代表允许明显的视觉穿模。
- 台面和地面参与物理碰撞；本规划器没有实现通用障碍搜索。初始姿态与默认台面配置经过物理执行验证，错误路径会因跟踪偏差停止，不能把它当作通用避障规划器。
- 蓝色水柱只是开度反馈的符号显示，不是流体仿真。温度与流量分别按阀角线性映射到 20～60℃、0～6 L/min，不求解液体、阀芯或热传导。
- 原始资产不改写。机器人底座及龙头主体到世界的安装固定关节保留，属于支座；机器人到龙头之间不创建任何抓取关节。摩擦系数、夹持力矩、刚体惯量和阀阻力仍是仿真参数，不能直接作为实物抓取能力的结论。

## 6. 目录与每个代码文件

```text
Open_Faucet/
├── dual_arm/                       原始机器人 USD、URDF 和 STL，保持原样
├── single_lever_faucet/            原始龙头资产及其原有演示/生成工具，保持原样
├── configs/
│   └── default.json               默认场景、TCP、初始姿态和操作序列
├── faucet_sim/                    本项目的功能模块
│   ├── __init__.py
│   ├── config.py
│   ├── geometry.py
│   ├── scene.py
│   ├── kinematics.py
│   ├── planning.py
│   ├── grasp.py
│   ├── collision.py
│   ├── filming.py                  录制专用机位、中文字幕与 MP4 编码
│   ├── visuals.py
│   └── runtime.py
├── scripts/
│   ├── run_sim.py                 正式仿真入口
│   ├── plan_only.py               不启动仿真应用的离线规划入口
│   ├── check_environment.py       环境检查入口
│   └── record_demo.py             多视角 1080p 仿真影片录制入口
├── tests/
│   └── test_planning.py           运动学、轨迹、正面抓取方向和碰撞配置回归检查
├── examples/                      公开保留的验收结果、配置和实际仿真截图
├── outputs/                       本机自动生成结果（不提交 Git）
├── run.sh                         使用已有 conda 环境的便捷启动器
├── requirements.txt               直接依赖版本记录
└── README.md                      本说明
```

| 代码文件 | 类/函数的职责、输入和输出 |
| --- | --- |
| `faucet_sim/__init__.py` | 标识仿真功能包 |
| `faucet_sim/config.py` | `load_config` 读取 JSON/解析资产路径；`validate` 校验输入角度、时间与数值，输出配置字典或抛异常 |
| `faucet_sim/geometry.py` | 齐次位姿、求逆、USD 四元数转换、旋转插值、五次时间函数和位姿误差；输入/输出为 NumPy 数组或误差标量 |
| `faucet_sim/scene.py` | `build_scene` 根据配置创建本地场景与世界锚点，输出 USD Stage，并保存 `scene.usda` |
| `faucet_sim/kinematics.py` | `Joint` 保存关节结构；`RightArmKinematics` 提供 FK/IK；`FaucetKinematics` 用阀角与把手局部抓取位姿输出世界 TCP 目标 |
| `faucet_sim/planning.py` | `Segment` 保存每段采样轨迹；`plan` 输出完整阶段列表；`save_plan` 保存 NPZ 和 JSON 摘要 |
| `faucet_sim/grasp.py` | `FrictionGrasp.begin/release` 仅登记接触状态；`observe` 测量相对滑移，`assert_no_attachment` 拒绝跨对象关节，全程不修改动力学 |
| `faucet_sim/collision.py` | `sliced_hulls` 裁剪原三角形，生成沿长度分段的凸碰撞体；`ContactMonitor` 订阅原生 PhysX 接触报告，逐步读取双方刚体、接触距离和冲量，穿透越界即停止 |
| `faucet_sim/filming.py` | `DemoFilm` 按操作阶段安排全景、俯视和特写机位，读取真实相机 RGB，叠加中文状态并用系统 FFmpeg 编码 |
| `faucet_sim/visuals.py` | `SimulationDisplay` 用实际阀角更新面板和水柱；`water_state` 输出理想水量/温度状态 |
| `faucet_sim/runtime.py` | `FaucetSimulation` 建立 PhysX、初始化执行器、按阶段执行、读取实际反馈、验收、截图和保存结果 |
| `scripts/run_sim.py` | 解析命令行，先创建 `SimulationApp` 再导入仿真模块；管理异常、保存与关闭应用 |
| `scripts/plan_only.py` | 在不创建仿真窗口的情况下解析资产并规划全部轨迹；输出场景和规划文件 |
| `scripts/check_environment.py` | 只读检测 Python、已有包、显卡与资产，不安装软件 |
| `scripts/record_demo.py` | 启动录制应用并执行完整物理回合，保存 1080p MP4、机位预览、分镜时间与实际验收结果 |
| `tests/test_planning.py` | 检查 FK 与 USD 零位一致、阀轴串联次序、轨迹连续/限位/速度、非法角度拒绝和上下夹持/碰撞开启 |
| `run.sh` | 固定工作目录，在已有环境中执行 Python 启动脚本 |

新增 Python 文件的模块、类、函数和关键计算均有中文注释。原资产目录中的原有工具仍保留其原始内容。

## 7. 输出与检查

默认每次运行输出到 `outputs/年月日_时分秒/`，可用 `--output` 指定固定目录。不要同时让两个进程写同一个输出目录。

| 文件 | 内容 |
| --- | --- |
| `scene.usda` | 本地资产引用和新建环境；静态场景，不是完整动作录像 |
| `effective_config.json` | 本次实际使用的完整配置，方便复现 |
| `planned_trajectory.npz` | 规划 `time`（秒）、`right_q`（N×7，弧度）、`faucet`（N×2，弧度）、`phase`；相邻段边界保留重复端点 |
| `plan_summary.json` | 阶段名称、阶段类型、时长、采样数和目标阀角 |
| `execution.csv` | 实测七轴角度、实际/目标阀角、TCP 误差、姿态误差、抓取状态、接触点数/冲量/穿透深度和理想水状态 |
| `result.json` | `passed`、抓取误差、每段目标/实际阀角、末态、最大跟踪误差及 `collision_check` 碰撞检查结果 |
| `frames/*.png` | 使用 `--capture-frames` 时保存的实际渲染结果 |

轨迹摘要是规划时间；实测日志还包含初始化、每段收敛等待和执行边界，所以实际仿真时间更长。`result.json` 的 `passed: true` 才表示完整物理流程通过；异常时退出码为 1。初始化失败也会留下失败标记，但可能还没有执行 CSV。

离线规划检查和回归测试：

```bash
conda activate dexonomy-isaac
python scripts/plan_only.py --output outputs/plan
python -m unittest discover -s tests -v
```

公开仓库在 `examples/` 中保留了完整验收结果（`validated_result.json`）、对应配置（`validated_config.json`）和实际渲染截图。原始运行日志、生成场景、缓存与调试文件位于本机 `outputs/`，不提交到 Git。

上述 `examples/validated_result.json` 是旧版固定抓取约束的历史验收：最大阀角误差约 0.098°、最大 TCP 误差约 0.993 mm。它不证明纯摩擦抓取成功。新版另外增加了“登记抓取不修改 USD、无跨对象关节、阀位置刚度为零”的回归检查，目前共六项离线检查。

`examples/grasp_closed_detail.png` 来自同一场景的带窗口短流程夹持检查；`examples/faucet_operation.png` 来自完整调温/开关流程。完整验收数据见 [examples/validated_result.json](examples/validated_result.json)。

## 8. 常见问题

- **中途暂停与继续**：点击 Isaac Sim 时间轴的 **Pause（暂停）**，再点击 **Play（播放）** 从原处继续。暂停时只刷新界面，不推进轨迹、收敛等待、仿真计时或验收。不要使用 **Stop（停止）** 代替暂停：Stop 会结束当前流程，需要重新启动脚本。

- **导入不到 `isaacsim`**：确认使用 `dexonomy-isaac`，不要用系统 Python。可直接使用 `bash run.sh ...`。
- **提示未能初始化 GPU**：在本机图形会话中执行 `nvidia-smi` 确认驱动可用，确保运行进程能访问显卡。无窗口模式也不是不需要显卡。
- **报目标不可达/样条限位/跟踪误差**：检查龙头摆放、初始姿态、TCP 和关节限制。先恢复默认配置复现，再逐项修改；不要通过关闭错误检查掩盖碰撞。
- **左右旋转和抬升方向不同于预想**：方向定义在龙头本体上，`faucet_yaw_deg` 会一起旋转阀轴与把手，抬升轴也会随调温载体旋转。
- **打开 `scene.usda` 后机器人没执行动作**：该文件只存场景。需要运行 `scripts/run_sim.py`，由控制器规划并发送动作。
- **看到 Isaac 自身旧接口/缓存提示**：本项目使用 `isaacsim.core.*` 接口；环境自带扩展可能仍打印弃用或缓存提示。应以 Python 异常和任务 `result.json` 判断流程是否完成。

接口参考：[Isaac Sim 5.0 Manipulators](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.robot.manipulators/docs/index.html)、[PhysX 关节说明](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/joints.html)。

## 9. 录制多视角视频

录制功能额外使用现有的 **Pillow**（中文画面标注）和系统 **FFmpeg/libx264**（H.264 MP4 编码），以及本机 Noto Sans CJK 字体。本机已具备，不会自动安装或下载。普通仿真不导入录制模块。

```bash
conda activate dexonomy-isaac
python scripts/record_demo.py --output outputs/videos/faucet_multiview
```

默认输出 **1920×1080、30 fps** 无声 MP4，包含片头、正面全景、上下夹持特写、俯视调温、侧视开关和撤离全景。视频中的角度和水状态来自实际物理反馈；相机与字幕不改变机器人动作或碰撞。录制可慢于实时，但成片按仿真时间正常播放。

- `faucet_multiview.mp4`：最终影片，可直接播放。
- `video_info.json`：分辨率、帧率、片长、各阶段起始时间。
- `preview_*.jpg`：机位预览，便于检查构图。
- `simulation/result.json`：本次实际物理验收结果。

如需先检查机位，可加 `--preview-only`；该模式保持初始物理状态，仅预览摄影机，不是完整动作视频。可用 `--width 1280 --height 720 --fps 30` 降低录制分辨率。

## 10. 无固定约束的摩擦实验

```bash
# 全部操作的摩擦基线：静摩擦 1.0、动摩擦 0.8、手指力矩上限 0.3 N·m。
bash run.sh --headless --quit-on-complete --capture-frames --output outputs/friction_baseline

# 摩擦系数为零的对照：相同机器人、碰撞、夹持参数，执行开阀及冷水侧旋转。
bash run.sh --headless --quit-on-complete --capture-frames \
  --config configs/friction_zero_control.json --output outputs/friction_zero_control
```

两组实验都没有任何机器人到龙头的关节或龙头位置控制。零摩擦对照可能失败；失败会保存 `passed: false`、真实阀角误差和失败截图，不通过增加隐藏约束补偿。

`result.json` 新增 `grasp_audit`（跨对象关节、驱动刚度、初始状态后的位置命令）、`friction_parameters`、`slip` 和 `contact_forces`。`execution.csv` 记录两指的法向力、摩擦力估计、接触点和相对滑移。力由接触/摩擦冲量除以报告覆盖的物理步长估计；离散接触峰值不等于实物稳态夹持力。

### 本机实际实验结果（2026-11-03）

| 检查项 | 摩擦基线（静 1.0 / 动 0.8） | 零摩擦对照 |
| --- | --- | --- |
| 机器人到龙头的抓取关节 | 无 | 无 |
| 龙头驱动位置刚度 / 初始化后位置命令 | 0 / 0 次 | 0 / 0 次 |
| 抬升到 20°的实测值 | 20.100° | 20.105° |
| 向冷水侧 −25°的实测值 | −24.847° | −19.647°，偏差 5.353°，验收失败 |
| 完整六个操作 | 全部满足 2° 终点容差 | 在第二个操作失败后停止 |
| 最大终点误差 | 1.795° | 5.353°（已执行的两个操作） |
| 最大相对平移 / 转动 | 0.470 mm / 1.914° | 0.448 mm / 5.415° |
| 测得累计摩擦冲量 | 145.888 N·s | 0 N·s |
| 最大报告穿透 | 0 mm | 0 mm |

这说明原来的夹持参数在去掉固定约束后可以完成这一组操作，精度比理想固定抓取低。零摩擦仍能带动部分转角，说明几何法向顶推/卡持也参与传力，因此准确名称是**无附着约束的接触与摩擦交互**，不能宣称“所有转动只来自切向摩擦、没有几何作用”。摩擦对于本次左右调温的跟踪精度有明显贡献。

结果副本：[摩擦基线](examples/friction_baseline_result.json)、[零摩擦对照](examples/friction_zero_control_result.json)、[对比摘要](examples/friction_comparison.json)。本次没有更改摩擦基线的夹持力矩或提高摩擦系数来补偿误差。本节结果来自另外实际执行的无约束实验。
