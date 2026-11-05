"""视觉位姿到龙头机构状态的适配；不读取仿真世界位姿或关节角。"""
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .geometry import pose_error, inverse


def rigid_transform(value):
    """拒绝非法、缩放或反射矩阵，避免把错误外参交给 IK。"""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('位姿必须是有限的 4×4 矩阵')
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-4)):
        raise ValueError('位姿不是合法的刚体变换')
    return matrix


def apply_pose_estimate(faucet, filename):
    """设置规划模型主体位姿，并在已知两轴结构内拟合视觉把手位姿。"""
    with open(filename, encoding='utf-8') as stream:
        data = json.load(stream)
    if data.get('source') != 'foundationpose_rgbd':
        raise ValueError('需要 estimate_pose.py 输出的 foundationpose_rgbd 结果')
    body = rigid_transform(data['objects']['body']['T_world_link'])
    handle = rigid_transform(data['objects']['handle']['T_world_link'])
    previous = faucet.body.copy()
    faucet.body = body
    def residual(angles):
        predicted = faucet.handle(*angles)
        return np.r_[(predicted[:3, 3] - handle[:3, 3]) / .005,
                     Rotation.from_matrix(handle[:3, :3] @ predicted[:3, :3].T).as_rotvec() / np.deg2rad(3)]
    result = least_squares(residual, np.deg2rad([0., 1.]),
                           bounds=(np.deg2rad([-45, 0]), np.deg2rad([45, 35])))
    distance, angle = pose_error(faucet.handle(*result.x), handle)
    if not result.success or distance > .015 or angle > np.deg2rad(10):
        faucet.body = previous
        raise ValueError(f'视觉位姿与关节结构不一致：{distance*1000:.1f} mm / {np.rad2deg(angle):.1f} deg')
    # 主体估计用来恢复关节状态；初始抓取必须对齐实际可见把手。
    # 将两刚体估计的结构残差吸收到规划主体的固定校正中，随后仍按同一关节链运动。
    # 不使用仿真真值，也不修改物理龙头的位姿。
    reconstructed = faucet.handle(*result.x)
    faucet.body = handle @ inverse(reconstructed) @ body
    faucet.handle_to_tcp = faucet.nominal_handle_to_tcp.copy()
    # 固定上指沿把手上方留出毫米级接近间隙，下指仍由受限力矩闭合形成夹持。
    faucet.handle_to_tcp[2, 3] += faucet.visual_clearance
    return result.x, {'source': data['source'], 'timestamp': data['timestamp'],
                      'grasp_alignment': 'observed_handle',
                      'visual_grasp_clearance_m': faucet.visual_clearance,
                      'estimated_body_world': body.tolist(),
                      'planning_body_world': faucet.body.tolist(),
                      'initial_valve_deg': np.rad2deg(result.x).tolist(),
                      'fit_position_error_mm': distance * 1000,
                      'fit_rotation_error_deg': float(np.rad2deg(angle))}
