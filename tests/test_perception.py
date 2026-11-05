"""用合成观测验证视觉到机构状态的坐标与关节拟合；不替代模型推理验收。"""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation
from faucet_sim.config import load_config
from faucet_sim.geometry import pose
from faucet_sim.kinematics import FaucetKinematics
from faucet_sim.perception import apply_pose_estimate, rigid_transform


class PerceptionTests(unittest.TestCase):
    def test_recover_nonzero_state_and_body_pose(self):
        config = load_config()
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, '/World/Faucet').GetPrim().GetReferences().AddReference(config['faucet_usd'])
        body = pose([.12, -.23, .48], Rotation.from_euler('z', 23, degrees=True).as_matrix())
        source = FaucetKinematics(stage, body, config)
        target = FaucetKinematics(stage, np.eye(4), config)
        angles = np.deg2rad([-22, 17])
        data = {'source': 'foundationpose_rgbd', 'timestamp': 1., 'objects': {
            'body': {'T_world_link': body.tolist()},
            'handle': {'T_world_link': source.handle(*angles).tolist()}}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'pose.json'
            path.write_text(json.dumps(data))
            result, report = apply_pose_estimate(target, path)
            np.testing.assert_allclose(result, angles, atol=1e-6)
            self.assertLess(report['fit_position_error_mm'], .001)
            data['objects']['handle']['T_world_link'][0][3] += .2
            path.write_text(json.dumps(data))
            before = target.body.copy()
            with self.assertRaises(ValueError):
                apply_pose_estimate(target, path)
            np.testing.assert_allclose(target.body, before)

    def test_reject_invalid_transform(self):
        for value in (np.zeros((4, 4)), np.diag([1, 1, -1, 1]), np.full((4, 4), np.nan)):
            with self.assertRaises(ValueError):
                rigid_transform(value)

    def test_grasp_uses_handle_observation_with_body_noise(self):
        """主体定位的毫米级残差不能直接变成初始夹爪定位偏差。"""
        config = load_config()
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, '/World/Faucet').GetPrim().GetReferences().AddReference(config['faucet_usd'])
        body = pose([0, 0, .48])
        source = FaucetKinematics(stage, body, config)
        noisy = body.copy()
        noisy[:3, 3] += [.001, .0002, -.0001]
        noisy[:3, :3] = Rotation.from_euler('xyz', [.46, -.75, -1.17], degrees=True).as_matrix()
        handle = source.handle(0, 0)
        data = {'source': 'foundationpose_rgbd', 'timestamp': 1., 'objects': {
            'body': {'T_world_link': noisy.tolist()}, 'handle': {'T_world_link': handle.tolist()}}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'pose.json'
            path.write_text(json.dumps(data))
            target = FaucetKinematics(stage, np.eye(4), config)
            angles, report = apply_pose_estimate(target, path)
            np.testing.assert_allclose(target.handle(*angles), handle, atol=1e-7)
            self.assertEqual(report['grasp_alignment'], 'observed_handle')


if __name__ == '__main__':
    unittest.main()
