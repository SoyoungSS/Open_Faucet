"""分阶段轨迹规划：转移、直线接近、闭爪、阀轴转动、松爪及撤离。"""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
from .geometry import interpolate, smoothstep, pose_error


@dataclass
class Segment:
    """一段已时间标定的轨迹，q 为 (N,7)，阀角为 (N,2)，单位均为弧度。"""
    name: str
    kind: str
    time: np.ndarray
    q: np.ndarray
    faucet: np.ndarray
    finger: np.ndarray
    target: np.ndarray


def plan(config, robot, faucet, initial_valve=None):
    """生成全部阶段；全部 IK、关节限位和速度检查成功后才允许执行。

    输入：场景配置和机器人/龙头运动学；输出：按顺序执行的 Segment 列表。
    """
    segments = []
    current_q = np.deg2rad(config['home_right_deg'])
    if np.any(current_q < robot.lower) or np.any(current_q > robot.upper):
        raise ValueError('home_right_deg 超出机器人关节限位')
    current_valve = np.zeros(2) if initial_valve is None else np.asarray(initial_valve, dtype=float).copy()
    if (current_valve.shape != (2,) or not np.isfinite(current_valve).all()
            or np.any(current_valve < np.deg2rad([-45, 0]))
            or np.any(current_valve > np.deg2rad([45, 35]))):
        raise ValueError('初始阀角超出关节限位')
    finger_open, finger_closed = np.deg2rad([config['finger_open_deg'], config['finger_closed_deg']])
    finger = finger_open

    def append(name, kind, duration, target_fn=None, q_goal=None, valve_goal=None, finger_goal=None):
        """按上一段终点连续规划下一段；超速时统一放慢整段。"""
        nonlocal current_q, current_valve, finger
        count = max(3, int(np.ceil(duration / config['knot_dt'])) + 1)
        knots = np.linspace(0, duration, count)
        q_knots, target_knots, valve_knots = [], [], []
        seed = current_q.copy()
        for knot_index, t in enumerate(knots):
            fraction = smoothstep(t / duration)
            valve = current_valve if valve_goal is None else current_valve + fraction * (valve_goal - current_valve)
            if q_goal is not None:
                seed = current_q + fraction * (q_goal - current_q)
                target = robot.forward(seed)
            elif target_fn is not None:
                target = target_fn(fraction, valve)
                # 相邻阶段复用同一个起点，避免重复 IK 带来微小的不连续。
                distance, angle = pose_error(robot.forward(seed), target)
                if knot_index != 0 or distance > .001 or angle > np.deg2rad(.3):
                    seed = robot.solve(target, seed)
            else:
                target = robot.forward(seed)
            q_knots.append(seed.copy())
            target_knots.append(target)
            valve_knots.append(valve.copy())
        # 单调 Hermite 插值避免普通三次样条在关节限位附近过冲。
        derivatives = PchipInterpolator(knots, q_knots).derivative()(knots)
        derivatives[[0, -1]] = 0.
        spline = CubicHermiteSpline(knots, q_knots, derivatives)
        times = np.linspace(0, duration, max(3, int(np.ceil(duration / config['physics_dt'])) + 1))
        speed = np.max(np.abs(spline(times, 1)))
        acceleration = np.max(np.abs(spline(times, 2)))
        scale = max(1., speed / config['max_joint_velocity'],
                    np.sqrt(acceleration / config['max_joint_acceleration'])) * 1.01
        duration *= scale
        times = np.arange(0, duration + config['physics_dt'] * .5, config['physics_dt'])
        if times[-1] < duration:
            times = np.r_[times, duration]
        # 时间放慢后重新参数化；不更改几何路径。
        q = spline(np.minimum(times / scale, knots[-1]))
        if np.any(q < robot.lower - 1e-6) or np.any(q > robot.upper + 1e-6):
            raise RuntimeError(f'{name} 样条越过关节限位')
        fractions = smoothstep(np.minimum(times / duration, 1.))
        valves = np.tile(current_valve, (len(times), 1)) if valve_goal is None else (
            current_valve + fractions[:, None] * (valve_goal - current_valve))
        fingers = np.full(len(times), finger) if finger_goal is None else finger + fractions * (finger_goal - finger)
        targets = np.array([target_fn(s, v) if target_fn else robot.forward(j)
                            for s, v, j in zip(fractions, valves, q)])
        errors = np.array([pose_error(robot.forward(j), target) for j, target in zip(q, targets)])
        if np.max(errors[:, 0]) > .0015 or np.max(errors[:, 1]) > np.deg2rad(.5):
            raise RuntimeError(f'{name} 轨迹插值误差过大：{errors.max(axis=0)}')
        segments.append(Segment(name, kind, times, q, valves, fingers, targets))
        current_q, current_valve, finger = q[-1], valves[-1], float(fingers[-1])

    grasp = faucet.target(*current_valve)
    pregrasp = grasp.copy()
    pregrasp[:3, 3] += config['pregrasp_distance'] * grasp[:3, 2]
    q_pre = robot.solve(pregrasp, current_q, restarts=True)
    append('转移到把手前方', 'transit', config['transit_duration'], q_goal=q_pre)
    append('沿夹爪轴接近把手', 'approach', config['approach_duration'],
           target_fn=lambda s, v: interpolate(pregrasp, grasp, s))
    append('闭合夹爪', 'grasp', config['grasp_duration'], finger_goal=finger_closed)
    for operation in config['operations']:
        append(operation['name'], 'operate', operation['duration'],
               target_fn=lambda s, v: faucet.target(*v),
               valve_goal=np.deg2rad([operation['yaw_deg'], operation['lift_deg']]))
    append('释放夹爪', 'release', config['grasp_duration'], finger_goal=finger_open)
    final = faucet.target(*current_valve)
    retreat = final.copy()
    retreat[:3, 3] += config['pregrasp_distance'] * final[:3, 2]
    append('沿夹爪轴撤离', 'retreat', config['retreat_duration'],
           target_fn=lambda s, v: interpolate(final, retreat, s))
    append('返回初始位置', 'home', config['transit_duration'], q_goal=np.deg2rad(config['home_right_deg']))
    return segments


def save_plan(segments, destination):
    """将完整可复查轨迹保存为 NPZ，并输出分阶段 JSON 摘要。"""
    destination = Path(destination)
    elapsed, time, q, faucet, phases = 0., [], [], [], []
    summary = []
    for index, segment in enumerate(segments):
        time.extend(elapsed + segment.time)
        q.extend(segment.q)
        faucet.extend(segment.faucet)
        phases.extend([index] * len(segment.time))
        elapsed += float(segment.time[-1])
        summary.append({'name': segment.name, 'kind': segment.kind, 'duration': float(segment.time[-1]),
                        'samples': len(segment.time), 'target_faucet_deg': np.rad2deg(segment.faucet[-1]).tolist()})
    np.savez_compressed(destination / 'planned_trajectory.npz', time=time, right_q=q, faucet=faucet, phase=phases)
    (destination / 'plan_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    return elapsed
