"""在独立 FoundationPose 环境中，对头部 RGB-D 快照执行双刚体注册。"""
import argparse
import json
import hashlib
from pathlib import Path
import sys
import numpy as np


def save_overlay(rgb, intrinsics, meshes, result, output):
    """叠加预测模型的投影轮廓与坐标轴，供人工检查，不代表精度验收。"""
    import cv2
    import trimesh
    from PIL import Image
    canvas = rgb.copy()
    height, width = rgb.shape[:2]
    colors = {'body': (255, 70, 50), 'handle': (40, 210, 255)}
    for name in ('body', 'handle'):
        mesh = trimesh.load(str(Path(meshes) / f'{name}.obj'), force='mesh')
        transform = np.asarray(result['objects'][name]['T_camera_link'])
        points = mesh.vertices @ transform[:3, :3].T + transform[:3, 3]
        projected = points @ intrinsics.T
        uv = projected[:, :2] / projected[:, 2:3]
        silhouette = np.zeros((height, width), dtype=np.uint8)
        for face in mesh.faces:
            if np.any(points[face, 2] <= .001):
                continue
            pixels = np.clip(uv[face], -10000, 10000).astype(np.int32)
            cv2.fillConvexPoly(silhouette, pixels, 255)
        contours, _ = cv2.findContours(silhouette, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, contours, -1, colors[name], 2)
        origin = transform[:3, 3]
        axes = np.vstack([origin, origin + .04 * transform[:3, :3].T])
        pixels = axes @ intrinsics.T
        pixels = np.clip(pixels[:, :2] / pixels[:, 2:3], -10000, 10000).astype(int)
        if np.all(axes[:, 2] > .001):
            for i, color in enumerate(((255, 0, 0), (0, 255, 0), (0, 80, 255)), 1):
                cv2.line(canvas, tuple(pixels[0]), tuple(pixels[i]), color, 2)
        cv2.putText(canvas, name, (12, 25 if name == 'body' else 50),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, colors[name], 2)
    Image.fromarray(canvas).save(output)


def main():
    """显式输入分割 mask，不从仿真真值读取位姿；不会自动安装依赖。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--foundationpose-root', required=True)
    parser.add_argument('--frame', required=True, help='head_camera/frame.npz')
    parser.add_argument('--meshes', required=True, help='export_pose_meshes.py 的输出目录')
    parser.add_argument('--body-mask', required=True, help='主体二值 PNG，非零为目标')
    parser.add_argument('--handle-mask', required=True, help='把手二值 PNG，非零为目标')
    parser.add_argument('--mask-source', default='external',
                        choices=['external', 'simulator_semantic_ground_truth'], help='记录 mask 来源')
    parser.add_argument('--output', required=True)
    parser.add_argument('--request-id', help='同次仿真调用的唯一请求标识')
    args = parser.parse_args()
    repo = Path(args.foundationpose_root).resolve()
    if not (repo / 'estimater.py').is_file():
        parser.error('FoundationPose 根目录缺少 estimater.py')
    sys.path.insert(0, str(repo))
    from PIL import Image
    import trimesh
    import nvdiffrast.torch as dr
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    from Utils import set_seed
    set_seed(0)
    data = np.load(args.frame, allow_pickle=False)
    rgb = np.ascontiguousarray(data['rgb'], dtype=np.uint8)
    depth = np.ascontiguousarray(data['depth'], dtype=np.float32)
    K = np.ascontiguousarray(data['K'], dtype=np.float32)
    if depth.shape != rgb.shape[:2] or K.shape != (3, 3):
        raise ValueError('RGB-D 或内参形状错误')
    scorer, refiner, context = ScorePredictor(), PoseRefinePredictor(), dr.RasterizeCudaContext()
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = {'source': 'foundationpose_rgbd', 'timestamp': float(data['timestamp']),
              'request_id': args.request_id,
              'frame_sha256': hashlib.sha256(Path(args.frame).read_bytes()).hexdigest(),
              'frame': str(Path(args.frame).resolve()),
              'mask_source': args.mask_source,
              'mask_paths': {'body': str(Path(args.body_mask).resolve()),
                             'handle': str(Path(args.handle_mask).resolve())}, 'objects': {}}
    for name, mask_path in [('body', args.body_mask), ('handle', args.handle_mask)]:
        mask = np.asarray(Image.open(mask_path).convert('L')) > 0
        if mask.shape != depth.shape or np.count_nonzero(mask & (depth > 0)) < 16:
            raise ValueError(f'{name} mask 形状不匹配或有效深度太少')
        mesh = trimesh.load(str(Path(args.meshes) / f'{name}.obj'), force='mesh')
        debug_dir = output.parent / ('debug_' + name)
        debug_dir.mkdir(exist_ok=True)
        estimator = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                                   mesh=mesh, scorer=scorer, refiner=refiner,
                                   glctx=context, debug=0, debug_dir=str(debug_dir))
        # NumPy 2 的 float64 标量会使官方裁剪偏移张量提升为 double；
        # 直径转为 Python float，保持与相机内参、CUDA 位姿的 float32 一致。
        estimator.diameter = float(estimator.diameter)
        estimate = np.asarray(estimator.register(K=K, rgb=rgb, depth=depth,
                                                 ob_mask=mask, iteration=5))
        if estimate.shape != (4, 4) or not np.isfinite(estimate).all():
            raise RuntimeError(f'{name} 位姿输出无效')
        result['objects'][name] = {'T_camera_link': estimate.tolist(),
                                   'T_world_link': (data['T_world_camera'] @ estimate).tolist()}
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    save_overlay(rgb, K, args.meshes, result, output.with_name(output.stem + '_overlay.png'))
    print(output)


if __name__ == '__main__':
    main()
