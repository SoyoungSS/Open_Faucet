"""导出 FoundationPose 刚体网格；保持 USD 刚体原点、米制单位，不居中。"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from pxr import Usd, UsdGeom

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from faucet_sim.config import load_config
from faucet_sim.geometry import world_pose, inverse


def export_meshes(config, output):
    """仅导出可见几何，主体包括固定出水嘴，把手单独导出。"""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.Open(config['faucet_usd'])
    root = str(stage.GetDefaultPrim().GetPath())
    manifest = {'units': 'meters', 'T_mesh_link': np.eye(4).tolist(), 'objects': {}}
    for name in ('body', 'handle'):
        link = stage.GetPrimAtPath(root + '/' + name)
        origin = inverse(world_pose(link))
        lines, offset = [], 1
        for prim in Usd.PrimRange(link):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            if mesh.ComputePurpose() not in ('default', 'render') or mesh.ComputeVisibility() == 'invisible':
                continue
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            transform = origin @ world_pose(prim)
            points = points @ transform[:3, :3].T + transform[:3, 3]
            lines.extend('v ' + ' '.join(f'{v:.9g}' for v in point) for point in points)
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
            cursor = 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                face = indices[cursor:cursor + count] + offset
                if mesh.GetOrientationAttr().Get() == 'leftHanded':
                    face = face[::-1]
                for i in range(1, count - 1):
                    lines.append(f'f {face[0]} {face[i]} {face[i+1]}')
                cursor += count
            offset += len(points)
        if offset == 1:
            raise RuntimeError(f'{name} 没有可导出的视觉网格')
        (output / f'{name}.obj').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        manifest['objects'][name] = {'mesh': f'{name}.obj', 'link': root + '/' + name,
                                     'vertices': offset - 1}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--output', default='outputs/pose_meshes')
    args = parser.parse_args()
    print(json.dumps(export_meshes(load_config(args.config), args.output), indent=2))
