"""离线回归检查：USD 坐标一致性、串联阀轴、限位及完整任务轨迹。"""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from pxr import UsdPhysics, UsdGeom
from faucet_sim.config import load_config, validate
from faucet_sim.geometry import world_pose, pose_error
from faucet_sim.kinematics import RightArmKinematics, FaucetKinematics
from faucet_sim.planning import plan
from faucet_sim.scene import build_scene
from faucet_sim.grasp import FrictionGrasp


class PlanningTests(unittest.TestCase):
    """不启动图形应用即可检验的规划核心；不替代实际物理验收。"""

    @classmethod
    def setUpClass(cls):
        """在临时目录生成测试场景，统一构造运动学对象和全部轨迹。"""
        cls.temp = tempfile.TemporaryDirectory()
        cls.config = load_config()
        cls.stage = build_scene(cls.config, Path(cls.temp.name) / 'scene.usda')
        cls.robot = RightArmKinematics(cls.stage, world_pose(cls.stage.GetPrimAtPath(
            '/World/Robot/arm_base_link')), cls.config['tcp_in_gripper'])
        cls.faucet = FaucetKinematics(cls.stage, world_pose(cls.stage.GetPrimAtPath(
            '/World/Faucet/body')), cls.config)
        cls.segments = plan(cls.config, cls.robot, cls.faucet)

    @classmethod
    def tearDownClass(cls):
        """释放 USD 场景及临时文件。"""
        cls.stage = None
        cls.temp.cleanup()

    def test_zero_pose_matches_usd(self):
        """逐级关节 FK 必须与资产零位的刚体世界坐标吻合。"""
        _, links = self.robot.forward(np.zeros(7), True)
        for path, actual in links.items():
            np.testing.assert_allclose(actual, world_pose(self.stage.GetPrimAtPath(path)), atol=2e-6)
        np.testing.assert_allclose(self.faucet.handle(0, 0), world_pose(self.stage.GetPrimAtPath(
            '/World/Faucet/handle')), atol=1e-7)

    def test_valve_serial_axes(self):
        """调温后抬升轴随载体转动；不能把两个世界轴旋转顺序写反。"""
        target = self.faucet.handle(np.pi/2, np.pi/6)
        point = (target @ np.array([0, .075, 0, 1]))[:3]
        pivot = self.config['faucet_position'] + np.array([0, 0, .192])
        np.testing.assert_allclose(point - pivot, [-.075*np.cos(np.pi/6), 0, .075*.5], atol=1e-7)

    def test_nonzero_visual_initial_state(self):
        """视觉初始角必须真正改变抓取目标，不能仍然隐式按零位抓取。"""
        initial = np.deg2rad([10., 5.])
        segments = plan(self.config, self.robot, self.faucet, initial_valve=initial)
        np.testing.assert_allclose(segments[0].faucet[0], initial)
        np.testing.assert_allclose(segments[1].target[-1], self.faucet.target(*initial), atol=1e-7)

    def test_trajectory_and_limits(self):
        """每一段都连续、有限、在关节限位内，并符合指定最大速度。"""
        for index, segment in enumerate(self.segments):
            self.assertTrue(np.isfinite(segment.q).all())
            self.assertTrue(np.all(segment.q >= self.robot.lower - 1e-6))
            self.assertTrue(np.all(segment.q <= self.robot.upper + 1e-6))
            speed = np.diff(segment.q, axis=0) / np.diff(segment.time)[:, None]
            self.assertLessEqual(np.max(np.abs(speed)), self.config['max_joint_velocity'] * 1.01)
            acceleration = np.diff(speed, axis=0) / ((np.diff(segment.time)[1:] +
                                                      np.diff(segment.time)[:-1]) / 2)[:, None]
            self.assertLessEqual(np.max(np.abs(acceleration)), self.config['max_joint_acceleration'] * 1.01)
            if index:
                np.testing.assert_allclose(segment.q[0], self.segments[index-1].q[-1], atol=1e-8)
            if segment.kind == 'operate':
                for i in range(0, len(segment.time), 10):
                    distance, angle = pose_error(self.robot.forward(segment.q[i]), self.faucet.target(*segment.faucet[i]))
                    self.assertLess(distance, .0015)
                    self.assertLess(angle, np.deg2rad(.5))
        np.testing.assert_allclose(self.segments[-1].q[-1], np.deg2rad(self.config['home_right_deg']), atol=1e-8)

    def test_invalid_angles_rejected(self):
        """越过资产物理限位的操作必须在启动前报错。"""
        for name, value in [('yaw_deg', 46), ('lift_deg', -1), ('lift_deg', 36)]:
            config = load_config()
            config['operations'][0][name] = value
            with self.assertRaises(ValueError):
                validate(config)

    def test_front_grasp_axes_and_collision_enabled(self):
        """防止回归到侧面抓取、水平夹持或屏蔽夹爪/把手碰撞的旧实现。"""
        c = self.config
        # 出水嘴局部 +Y 就是龙头前方；右肩与把手对齐，基座位于前方。
        self.assertGreater(c['robot_position'][1], c['faucet_position'][1] + .4)
        frame = self.faucet.target(0, 0)
        np.testing.assert_allclose(frame[:3, 1], [0, 0, -1], atol=1e-7)
        np.testing.assert_allclose(-frame[:3, 2], [0, -1, 0], atol=1e-7)
        for name in ['arm_r_link_gripper', 'arm_r_link_finger']:
            root = self.stage.GetPrimAtPath('/World/Robot/' + name)
            relation = root.GetRelationship('physics:filteredPairs')
            self.assertFalse(relation and relation.GetTargets())
            old_collision_root = self.stage.GetPrimAtPath(str(root.GetPath()) + '/collisions')
            self.assertFalse(old_collision_root.IsInstance())
            collision_root = self.stage.GetPrimAtPath(str(root.GetPath()) + '/contact_hulls')
            from pxr import Usd
            colliders = [p for p in Usd.PrimRange(collision_root) if p.HasAPI(UsdPhysics.CollisionAPI)]
            self.assertGreater(len(colliders), 20)
            for prim in colliders:
                self.assertTrue(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
                self.assertEqual(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), 'convexHull')
                self.assertTrue(prim.IsA(UsdGeom.Mesh))
                points = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                self.assertLessEqual(float(np.ptp(points[:, 2])), .002001)

    def test_friction_grasp_does_not_create_attachment(self):
        """登记与释放抓取不能改变 USD；龙头驱动在场景层中也必须被动。"""
        grasp = FrictionGrasp(self.stage, self.robot, self.faucet)
        before = self.stage.GetRootLayer().ExportToString()
        grasp.begin(self.segments[1].q[-1], np.zeros(2))
        self.assertTrue(grasp.active)
        self.assertEqual(grasp.assert_no_attachment(), [])
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        state = grasp.observe(self.segments[1].q[-1], np.zeros(2))
        self.assertLess(state['slip_mm'], 1e-6)
        self.assertLess(state['slip_deg'], 1e-6)
        grasp.release()
        self.assertFalse(grasp.active)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        for name in ['temperature_yaw', 'flow_lift']:
            drive = UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath('/World/Faucet/joints/' + name), 'angular')
            self.assertEqual(drive.GetStiffnessAttr().Get(), 0.)


if __name__ == '__main__':
    unittest.main()
