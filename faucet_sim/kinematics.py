"""直接读取 USD 关节坐标系的正逆运动学，不用外部机器人模型包。"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from pxr import UsdPhysics
from .geometry import gf_pose, inverse, pose, pose_error


@dataclass
class Joint:
    """单个 USD 关节：两侧局部坐标系、轴、弧度限位及刚体路径。"""
    name: str
    parent: str
    child: str
    frame0: np.ndarray
    frame1: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    movable: bool


def read_joint(prim):
    """将一个 USD Joint 转换为可计算的数据对象。"""
    joint = UsdPhysics.Joint(prim)
    parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
    movable = prim.IsA(UsdPhysics.RevoluteJoint)
    revolute = UsdPhysics.RevoluteJoint(prim) if movable else None
    axis = np.eye(3)['XYZ'.index(revolute.GetAxisAttr().Get())] if movable else np.zeros(3)
    return Joint(prim.GetName(), str(parents[0]) if parents else '', str(children[0]),
                 gf_pose(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                 gf_pose(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()), axis,
                 np.deg2rad(revolute.GetLowerLimitAttr().Get()) if movable else 0.,
                 np.deg2rad(revolute.GetUpperLimitAttr().Get()) if movable else 0., movable)


def joint_transform(joint, angle=0.):
    """输入关节角（弧度），输出父刚体到子刚体的变换。"""
    rotation = Rotation.from_rotvec(joint.axis * angle).as_matrix() if joint.movable else np.eye(3)
    return joint.frame0 @ pose(rotation=rotation) @ inverse(joint.frame1)


class RightArmKinematics:
    """右臂七自由度运动学；末端为原资产的夹爪基座，TCP 单独标定。"""

    def __init__(self, stage, base_pose, tcp_offset):
        """stage 为组合场景；base_pose 为机器人基座世界变换。"""
        self.base = base_pose
        self.tcp = pose(tcp_offset)
        names = [f'arm_r_joint_{i}' for i in range(1, 8)] + ['arm_r_joint_tf', 'arm_r_joint_gripper']
        self.chain = [read_joint(stage.GetPrimAtPath('/World/Robot/joints/' + name)) for name in names]
        self.names = names[:7]
        self.lower = np.array([j.lower for j in self.chain[:7]])
        self.upper = np.array([j.upper for j in self.chain[:7]])

    def forward(self, angles, return_links=False):
        """输入 7 个关节角；返回世界 TCP 位姿，或 TCP 及所有链上刚体位姿。"""
        transform, links = self.base.copy(), {}
        for index, joint in enumerate(self.chain):
            transform = transform @ joint_transform(joint, angles[index] if index < 7 else 0.)
            links[joint.child] = transform.copy()
        tcp = transform @ self.tcp
        return (tcp, links) if return_links else tcp

    def solve(self, target, seed, restarts=False):
        """有界最小二乘 IK；位置误差<0.5 mm、姿态误差<0.3°才返回。

        输入世界目标位姿、上一时刻关节角；输出连续的七轴解。
        首次转移可启用确定性多起点；失败抛异常，不用假解继续播放。
        """
        seed = np.clip(seed, self.lower + 1e-7, self.upper - 1e-7)

        def residual(q):
            actual = self.forward(q)
            rotation = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            return np.r_[actual[:3, 3] - target[:3, 3], 0.20 * rotation, 0.00005 * (q - seed)]

        starts = [seed]
        if restarts:
            rng = np.random.default_rng(7)
            starts += [rng.uniform(self.lower, self.upper) for _ in range(18)]
        candidates = []
        for start in starts:
            result = least_squares(residual, start, bounds=(self.lower + 1e-8, self.upper - 1e-8),
                                   max_nfev=160, ftol=1e-9, xtol=1e-9, gtol=1e-9)
            distance, angle = pose_error(self.forward(result.x), target)
            if distance < 0.0005 and angle < np.deg2rad(0.3):
                candidates.append(result.x)
                if not restarts:
                    return result.x
        if candidates:
            return min(candidates, key=lambda q: np.linalg.norm(q - seed))
        raise RuntimeError(f'目标不可达：位置={target[:3, 3].round(4).tolist()}，请调整场景或抓取位姿')


class FaucetKinematics:
    """读取水龙头两级关节结构；任意角度下的把手位姿就是规划输入。"""

    def __init__(self, stage, body_pose, config):
        """输入主体世界位姿与配置；抓取点定义在把手局部坐标系。"""
        self.body = body_pose
        self.joints = [read_joint(stage.GetPrimAtPath('/World/Faucet/joints/' + name))
                       for name in ['temperature_yaw', 'flow_lift']]
        # 夹爪 +Y 为开合方向，-Z 为手指伸出方向。
        # 默认将 +Y 对齐把手 -Z（上下夹），-Z 对齐把手 -Y（沿把手轴向接近）。
        self.handle_to_tcp = pose(config['grasp_in_handle'], Rotation.from_euler(
            'xyz', config['gripper_rpy_in_handle_deg'], degrees=True).as_matrix())
        self.nominal_handle_to_tcp = self.handle_to_tcp.copy()
        self.visual_clearance = float(config.get('visual_grasp_clearance_m', .001))
        if not np.isfinite(self.visual_clearance) or not 0 <= self.visual_clearance <= .005:
            raise ValueError('visual_grasp_clearance_m 必须在 [0,0.005] 米内')

    def handle(self, yaw, lift):
        """输入调温/抬升角（弧度），输出把手世界位姿。"""
        return self.body @ joint_transform(self.joints[0], yaw) @ joint_transform(self.joints[1], lift)

    def target(self, yaw, lift):
        """输入两个角度，输出保持抓取相对变换的 TCP 世界目标。"""
        return self.handle(yaw, lift) @ self.handle_to_tcp
