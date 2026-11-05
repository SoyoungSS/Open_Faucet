# 头部固定相机与 FoundationPose 接入

使用用户指定的较高相机 `arm_base_link_d435i_2`。相机挂在
`/World/Robot/arm_base_link_d435i_2/RGBDCamera`，随机器人安装位运动，不读取龙头位置自动瞄准。
原始 URDF/USD 资产不修改。较低的 `d435i_1` 不参与。

当前实现是**单帧视觉初始化规划**：RGB-D 快照 → 两个刚体位姿 → 两轴状态拟合 → 现有 IK/轨迹。
不是在线视觉闭环；仿真真实阀角与接触仍用于原有验收和失败停止，不用于生成视觉模式的规划目标。
已在本机安装官方 FoundationPose 并完成头部 RGB-D 的真实模型推理；本机安装与结果见下文。

## 一次启动：在同一仿真中采集、估计并执行

```bash
bash run.sh --headless --visual-pipeline --quit-on-complete \
  --output outputs/visual_pipeline_run
```

该模式在初始化后保持机械臂初始姿态，由较高头部相机重新采集 RGB-D 与仿真语义真值 mask，
在独立 FoundationPose 环境运行本次双刚体注册，验证位姿与机构约束，随后规划并执行完整流程。
不需要先生成位姿文件，不能与 `--pose-estimate` 或 `--head-camera-only` 混用。
推理等待期间暂停物理、刷新界面；规划完成后恢复同一个世界，不重置机器人或龙头状态。

每次请求生成唯一目录 `pipeline/<request_id>/`，包含相机输入、CAD 网格、
`inference.log`、`estimated_pose.json` 和投影叠加图。
`pipeline.json` 记录采集、推理、校验、规划、执行及完成/失败阶段，最终 `result.json` 包含流程信息。
推理结果必须匹配请求 ID、图像绝对路径、SHA256 和采样时间戳；不回退到旧位姿或真值规划。
超时、子进程失败、非法位姿、结构残差过大或 IK 失败均停止，不进入机械臂操作阶段。

默认推理超时 300 秒，可用 `--pose-timeout 180` 调整；
`--foundationpose-root` 可指定其他已安装的官方代码目录。执行仍要求初始关闭状态。
该模式使用真值 mask，尚未包含自动分割、操作后视觉复核或操作中的在线视觉反馈。

### 视觉抓取对齐与接近间隙

初始抓取直接对齐本次估计的把手位姿。主体位姿仍用于拟合初始阀角，
两刚体观测之间的残差吸收到规划主体的固定校正中，再沿同一串联关节链生成操作路径。
这一校正只使用视觉观测，不修改物理龙头，也不读取真值来修正规划。
默认 `visual_grasp_clearance_m: 0.001` 为固定上指保留 1 mm 接近间隙；
下指仍靠原有受限力矩闭合。普通非视觉模式的抓取变换不受此项影响。
接近或闭爪时 TCP 跟踪偏差超过 5 mm / 5°会提前停止；原抓取与终点验收阈值不放宽。

用户运行 `outputs/20261105_111702/` 的失败发生在接近阶段：
把手被顶到约 19.7°，TCP 跟踪偏差约 27 mm，随后抓取检查报 14.73 mm / 27.71°。
这与前面的 Semantics/DLSS 警告无关。修复后使用该次原始位姿文件重放，
`outputs/grasp_clearance_replay/result.json` 完整通过六个操作：
抓取误差约 0.874 mm / 0.650°，最大终点误差约 1.822°，释放后开度约 0°。
19 项离线回归检查通过。该结果是原故障输入的验证，不代表任意视觉误差下都能成功。
另以带窗口 `--visual-pipeline --quit-on-complete` 重新采集、推理并执行，
`outputs/visual_gui_grasp_fix/result.json` 也完整通过：抓取误差约 0.792 mm / 0.787°，
六个操作最大终点误差约 1.598°，释放后开度约 0°；接近阶段最大 TCP 跟踪偏差约 0.680 mm。

本机已完成一次同次世界端到端实测，结果位于 `outputs/visual_pipeline_validation/`：
六个操作全部通过，最大终点误差约 1.648°，释放后开度约 0°。
采集后推理、校验、规划阶段仿真计时保持不变，再恢复同一个世界执行。
另以 `--pose-timeout 0.01` 做实际超时注入，确认没有生成轨迹、没有操作终点、未进入执行阶段，
结果位于 `outputs/visual_pipeline_timeout_check/`。18 项回归检查通过。
另在 `outputs/visual_pipeline_timeout_exit_check/` 确认失败退出码为 1；
脚本通过 Kit 官方退出接口传递错误码，避免快速关闭时误报成功。
本次完整数据、请求信息、输入与推理输出见 `result.json`、`pipeline.json` 和 `REPORT.md`。

## 1. 获取头部 RGB-D 与调试 mask

```bash
bash run.sh --headless --head-camera-only --debug-pose-masks \
  --quit-on-complete --output outputs/head_camera_check
```

输出 `head_camera/rgb.png`、`depth.npy`、`frame.npz`、`camera.json`。
`frame.npz` 含 RGB、米制光轴深度、K、`T_world_camera`、仿真采样时间和渲染帧标识。
无效深度为零。光学坐标为 X 向右、Y 向下、Z 向前。

`--debug-pose-masks` 额外生成 `body_mask_gt.png` 和 `handle_mask_gt.png`。
它们是仿真语义真值，仅隔离验证位姿估计链路；不是图像分割模型的结果。
正式感知实验应换成从相同 RGB 帧得到的分割 mask，并单独评测分割误差。

`configs/default.json` 的 `head_camera` 配置控制分辨率、像素焦距、镜头偏置及光学相对旋转。
默认 640×480、fx=fy=460；镜头位于外壳局部 `[0,0,0.002]`，光学轴与 link 轴对齐。
这些是明确的**仿真假设**，不是实机 D435i 标定参数。USD 光学轴转换由代码处理。
已实际检查该安装方向能看见龙头主体与把手；尚未验证整段操作的遮挡情况。

## 2. 导出刚体网格

```bash
conda run -n dexonomy-isaac python scripts/export_pose_meshes.py \
  --output outputs/pose_meshes
```

导出 `body.obj`（含固定出水嘴）及 `handle.obj`。两者都以对应 USD 刚体坐标为原点，
单位米，不进行居中或缩放，因此模型坐标到刚体坐标的转换为单位矩阵。
`manifest.json` 保存约定。不得把相机画面中其他活动部件合并进主体刚体模型。

## 3. 在独立环境运行 FoundationPose

参考 [官方仓库](https://github.com/NVlabs/FoundationPose) 准备代码、环境与预训练权重。
不要将其依赖直接覆盖安装进 Isaac Sim 环境。下面的 `python` 必须来自 FoundationPose 环境：

```bash
python /home/shendongran/code/Open_Faucet/scripts/estimate_pose.py \
  --foundationpose-root /path/to/FoundationPose \
  --frame /home/shendongran/code/Open_Faucet/outputs/head_camera_check/head_camera/frame.npz \
  --meshes /home/shendongran/code/Open_Faucet/outputs/pose_meshes \
  --body-mask /home/shendongran/code/Open_Faucet/outputs/head_camera_check/head_camera/body_mask_gt.png \
  --handle-mask /home/shendongran/code/Open_Faucet/outputs/head_camera_check/head_camera/handle_mask_gt.png \
  --mask-source simulator_semantic_ground_truth \
  --output /home/shendongran/code/Open_Faucet/outputs/estimated_pose.json
```

脚本为主体和把手分别调用官方 `register`，输出相机系、世界系位姿，并保存输入 mask 路径。
转换为 `T_world_link = T_world_camera @ T_camera_link`。
模型形状、对称性、遮挡、深度和分割误差都会影响结果；接口不会虚构可用置信度。

## 4. 用视觉结果验证规划，再执行

```bash
conda run -n dexonomy-isaac python scripts/plan_only.py \
  --pose-estimate outputs/estimated_pose.json --output outputs/visual_plan

bash run.sh --pose-estimate outputs/estimated_pose.json \
  --realtime --quit-on-complete --output outputs/visual_execution
```

规划器使用估计主体世界位姿，并根据估计把手位姿拟合 yaw/lift；不再强制从零角生成抓取目标。
拟合受关节限位约束；结构残差超过 15 mm 或 10°时拒绝规划。
这些是接入保护阈值，不代表该误差下能可靠抓取。

离线规划支持非零初始角。当前物理执行仍将龙头初始化为关闭状态，因此快照执行只接受
拟合初始角不超过 3°的结果；它针对第一阶段“初始关闭、改变龙头摆放”的实验。
采样和执行必须使用同一机器人、龙头摆放、相机与配置，场景不能发生变化；不应重放旧场景的估计结果。

`result.json` 的 `perception` 标明 `snapshot_initialization`、
`ground_truth_evaluation: true`、`online_visual_feedback: false`。
未传入 `--pose-estimate` 时继续使用原有真值规划模式。

## 后续在线闭环

需要将采集和推理改为持续通信，检测过期帧/跟踪失效，按当前实测右臂状态重新规划短段轨迹并平滑衔接；
同时支持非零初始物理状态，并将抓取与停止判断中的真值依赖替换为视觉/接触反馈。
应先完成当前快照模式的真实 FoundationPose 推理、网格叠加和随机摆放执行验证，再扩展这一步。

## 本机安装与首次推理结果（2026-11-05）

本机环境位于 `outputs/foundationpose_install/`，不要在清理普通输出时删除该目录。
`venv/` 只读复用 `dexonomy-isaac` 的 Python 3.11、PyTorch 2.7.0+cu128 与 torchvision，
其余新增或覆盖依赖安装在该虚拟环境中，未修改原 Isaac 环境的包。
CUDA 12.8、GCC 13 和构建依赖位于 `toolchain/`，不需要全局安装 CUDA。
已为 RTX 5060 Laptop（sm_120）构建 PyTorch3D 0.7.9、nvdiffrast 0.4.0，以及 FoundationPose mycpp。
已实际验证 CUDA 矩阵运算、nvdiffrast 三角形渲染和双刚体注册。

源码来自 [FoundationPose 官方仓库](https://github.com/NVlabs/FoundationPose)、
[PyTorch3D 官方仓库](https://github.com/facebookresearch/pytorch3d)、
[nvdiffrast 官方仓库](https://github.com/NVlabs/nvdiffrast)；权重来自 FoundationPose README 的公开链接。
`outputs/foundationpose_install/installation.json` 保存实际版本、源码 commit 和权重 SHA256。

从项目根目录复现本次估计，无需手动激活环境：

```bash
bash scripts/foundationpose_python.sh scripts/estimate_pose.py \
  --foundationpose-root outputs/foundationpose_install/FoundationPose \
  --frame outputs/head_camera_check/head_camera/frame.npz \
  --meshes outputs/pose_meshes \
  --body-mask outputs/head_camera_check/head_camera/body_mask_gt.png \
  --handle-mask outputs/head_camera_check/head_camera/handle_mask_gt.png \
  --mask-source simulator_semantic_ground_truth \
  --output outputs/foundationpose_result/estimated_pose.json
```

包装脚本提供正确的 CUDA/GCC/头文件路径。接口对 NumPy 2 的直径标量做 Python float 转换，
避免官方裁剪代码产生 float32/float64 混用；官方网络结构、权重和位姿注册算法均未修改。

实际输出为 `outputs/foundationpose_result/estimated_pose.json`（相机/世界系 4×4 位姿）、
`estimated_pose_overlay.png`（主体红色、把手青色投影轮廓）和 `evaluation.json`。

| 对象 | 世界位置（米） | 相对标称位置差 | 相对标称姿态差 |
| --- | --- | --- | --- |
| 主体 | `[0.000903, -0.000535, 0.480520]` | 1.17 mm | 1.04° |
| 把手 | `[0.000057, -0.000419, 0.671895]` | 0.44 mm | 0.18° |

两轴拟合为 yaw≈0.73°、lift≈0°，结构残差约 1.30 mm、0.57°。
参考来自初始关闭的标称 USD 场景，不是采样时动态真值测量；参考信息仅用于事后比较，没有输入模型。
本次采用仿真语义真值 mask 和理想渲染深度，只验证这一帧的位姿链路。
尚未验证自动分割、带噪声深度、随机摆放、全过程遮挡或视觉驱动的物理执行成功率。

## 单次视觉初始化的完整物理执行验证（2026-11-05）

使用上述真实模型输出，以默认配置、初始关闭状态执行了完整流程。
在接近、双指接触夹持之后，六个阀角操作全部通过原有 2° 终点容差，
随后完成释放、撤离和返回初始位置；`outputs/foundationpose_execution_retry/result.json` 为 `passed: true`。

| 操作 | 实测 yaw / lift（度） | 最大终点误差 |
| --- | --- | --- |
| 打开到 20° | −0.228 / 20.953 | 0.953° |
| 转冷水侧 −25° | −25.493 / 21.259 | 1.259° |
| 转热水侧 30° | 30.125 / 20.403 | 0.403° |
| 增大开度到 32° | 31.757 / 32.417 | 1.757° |
| 减小开度到 12° | 31.974 / 12.342 | 1.974° |
| 居中并关闭 | 1.774 / 0.671 | 1.774° |

释放并返回后，实测开度约 0°，调温角约 1.778°；最大机械臂 TCP 跟踪误差 1.303 mm，
最大把手相对滑移 0.632 mm / 2.654°。物理审计确认没有跨对象附着关节，龙头驱动位置刚度为零，
初始化后龙头位置命令为零。原摩擦、夹持力矩和验收阈值均未改变。

首次带关键帧的执行因 SIGTERM（143）中断在调温阶段，未完成验收，记录保留在
`outputs/foundationpose_execution/`；本节通过结果来自随后在持久终端内重新完成的全流程。

结果目录含输入位姿副本 `input_estimated_pose.json`、完整 `result.json`、`execution.csv`、
日志 `run.log`、摘要 `validation_summary.json`、曲线 `tracking.png` 与详细 `REPORT.md`。
最大误差距 2° 容差只剩约 0.026°，因此不能据此声称具有充分鲁棒性。
这是固定摆放、真值 mask、理想深度、单次视觉快照的一次完整验证；随机摆放及在线反馈仍待验证。

复现命令（使用相同场景配置与位姿文件）：

```bash
bash run.sh --headless --pose-estimate outputs/foundationpose_result/estimated_pose.json \
  --quit-on-complete --output outputs/visual_execution
```
