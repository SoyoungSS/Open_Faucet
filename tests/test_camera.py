"""检查相机安装链、光学投影方向与数据校验，不需要启动 RTX。"""
import unittest
import numpy as np
from pxr import Usd, UsdGeom
from faucet_sim.camera import CAMERA_PATH, MOUNT_PATH, create_head_camera, camera_settings
from faucet_sim.config import load_config
from faucet_sim.geometry import world_pose, inverse


class CameraTests(unittest.TestCase):
    """保证较高相机取景遵循安装关节，而不是读取目标后自动朝向。"""

    def test_mount_and_forward_projection(self):
        config = load_config()
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, '/World/Robot').GetPrim()
        root.GetReferences().AddReference(config['robot_usd'])
        create_head_camera(stage, config)
        optical = world_pose(stage.GetPrimAtPath(CAMERA_PATH)) @ np.diag([1., -1., -1., 1.])
        mount = world_pose(stage.GetPrimAtPath(MOUNT_PATH))
        np.testing.assert_allclose((inverse(mount) @ optical)[:3, 3], [0, 0, .002], atol=1e-8)
        np.testing.assert_allclose(optical[:3, :3], mount[:3, :3], atol=1e-8)
        # 高位相机在机器人基座坐标中向前（+X）并向下看。
        self.assertGreater(optical[0, 2], .6)
        self.assertLess(optical[2, 2], -.7)
        before = optical.copy()
        config['faucet_position'] = [100, 100, 100]
        create_head_camera(stage, config)
        np.testing.assert_allclose(before, world_pose(stage.GetPrimAtPath(CAMERA_PATH)) @ np.diag([1., -1., -1., 1.]))

    def test_invalid_intrinsics(self):
        for override in ({'resolution': [640.5, 480]}, {'focal_pixels': 0},
                         {'position_in_mount': [float('nan'), 0, 0]}):
            with self.assertRaises(ValueError):
                camera_settings({'head_camera': override})


if __name__ == '__main__':
    unittest.main()
