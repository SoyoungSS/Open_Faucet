"""头部较高 D435i 的虚拟 RGB-D 相机；光学系为右、下、前。"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import pose, world_pose


MOUNT_PATH = '/World/Robot/arm_base_link_d435i_2'
CAMERA_PATH = MOUNT_PATH + '/RGBDCamera'


def camera_settings(config):
    """内参与镜头偏置为仿真假设，可覆盖；不是 D435i 实机标定。"""
    settings = dict(resolution=[640, 480], focal_pixels=460.,
                    position_in_mount=[0., 0., .002], optical_rpy_deg=[0., 0., 0.])
    settings.update(config.get('head_camera', {}))
    size = np.asarray(settings['resolution'])
    if size.shape != (2,) or not np.isfinite(size).all() or np.any(size <= 0) or np.any(size != size.astype(int)):
        raise ValueError('head_camera.resolution 必须是两个正整数')
    if not np.isfinite(settings['focal_pixels']) or settings['focal_pixels'] <= 0:
        raise ValueError('head_camera.focal_pixels 必须为正有限值')
    for key in ('position_in_mount', 'optical_rpy_deg'):
        values = np.asarray(settings[key], dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            raise ValueError(f'head_camera.{key} 必须是三个有限值')
    return settings


def create_head_camera(stage, config):
    """只在场景覆盖层挂载 Camera，不改变原始资产或自动瞄准目标。"""
    from pxr import UsdGeom, Gf
    from .scene import set_transform
    if not stage.GetPrimAtPath(MOUNT_PATH):
        raise ValueError(f'找不到头部相机安装位：{MOUNT_PATH}')
    settings = camera_settings(config)
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    optical_rotation = Rotation.from_euler('xyz', settings['optical_rpy_deg'], degrees=True).as_matrix()
    # USD Camera 为右、上、后；转换为右、下、前的视觉光学系。
    set_transform(camera.GetPrim(), pose(settings['position_in_mount'],
                  optical_rotation @ np.diag([1., -1., -1.])))
    width, height = settings['resolution']
    focal = 20.  # USD Camera 的焦距与光圈使用同一单位，投影由比值决定。
    camera.CreateFocalLengthAttr(focal)
    camera.CreateHorizontalApertureAttr(focal * width / settings['focal_pixels'])
    camera.CreateVerticalApertureAttr(focal * height / settings['focal_pixels'])
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 10.))
    camera.CreateFStopAttr(0.)
    return camera


class HeadRGBDCamera:
    """在 World reset 后创建传感器，按同一渲染帧导出图像与光学外参。"""

    def __init__(self, config, debug_masks=False):
        from isaacsim.sensors.camera import Camera
        settings = camera_settings(config)
        self.sensor = Camera(prim_path=CAMERA_PATH, resolution=tuple(settings['resolution']))
        self.sensor.initialize()
        self.sensor.add_distance_to_image_plane_to_frame()
        self.debug_masks = debug_masks
        if debug_masks:
            import omni.usd
            from isaacsim.core.utils.semantics import add_update_semantics
            stage = omni.usd.get_context().get_stage()
            for name in ('body', 'handle'):
                add_update_semantics(stage.GetPrimAtPath('/World/Faucet/' + name), 'faucet_' + name)
            self.sensor.add_semantic_segmentation_to_frame()

    def save(self, stage, folder):
        """保存无损 RGB、米制光轴深度、K、世界光学位姿和采样时间。"""
        from PIL import Image
        frame = self.sensor.get_current_frame(clone=True)
        rgba, depth = frame.get('rgb'), frame.get('distance_to_image_plane')
        if rgba is None or depth is None or not np.size(rgba) or not np.size(depth):
            raise RuntimeError('头部相机尚未获得同步 RGB-D 帧')
        rgb = np.asarray(rgba, dtype=np.uint8)[..., :3]
        depth = np.asarray(depth, dtype=np.float32).copy()
        if depth.shape != rgb.shape[:2]:
            raise RuntimeError('RGB 与深度分辨率不一致')
        depth[~np.isfinite(depth) | (depth <= 0) | (depth > 10.)] = 0
        if not np.any(depth):
            raise RuntimeError('头部相机没有有效深度')
        transform = world_pose(stage.GetPrimAtPath(CAMERA_PATH)) @ np.diag([1., -1., -1., 1.])
        intrinsics = np.asarray(self.sensor.get_intrinsics_matrix())
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(folder / 'frame.npz', rgb=rgb, depth=depth, K=intrinsics,
                            T_world_camera=transform, timestamp=frame['rendering_time'],
                            rendering_frame_json=json.dumps(frame['rendering_frame']))
        Image.fromarray(rgb).save(folder / 'rgb.png')
        np.save(folder / 'depth.npy', depth)
        if self.debug_masks:
            segmentation = frame.get('semantic_segmentation')
            if not isinstance(segmentation, dict):
                raise RuntimeError('语义 mask 尚未生成')
            labels = segmentation['info']['idToLabels']
            for name in ('body', 'handle'):
                ids = [int(key) for key, label in labels.items()
                       if label.get('class') == 'faucet_' + name]
                mask = np.isin(segmentation['data'], ids)
                if not mask.any():
                    raise RuntimeError(f'{name} 在头部相机中没有可见语义像素')
                Image.fromarray(mask.astype(np.uint8) * 255).save(folder / f'{name}_mask_gt.png')
        (folder / 'camera.json').write_text(json.dumps({
            'mount_path': MOUNT_PATH, 'camera_path': CAMERA_PATH,
            'optical_axes': 'x right, y down, z forward', 'depth_units': 'meters',
            'depth_type': 'distance_to_image_plane', 'K': intrinsics.tolist(),
            'T_world_camera': transform.tolist(), 'timestamp': float(frame['rendering_time']),
            'rendering_frame': frame['rendering_frame'],
            'calibration': 'synthetic pinhole; mount-to-optical transform assumed, not calibrated',
            'mask_source': 'simulator_semantic_ground_truth' if self.debug_masks else None,
        }, ensure_ascii=False, indent=2), encoding='utf-8')
        return folder
