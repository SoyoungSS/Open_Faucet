"""同一次世界内采集、独立进程推理、校验和规划；推理期间冻结物理。"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import numpy as np


def validate_response(filename, frame, request_id):
    """只接受本次请求、当前图像的结果，拒绝旧文件和异常时间戳。"""
    data = json.loads(Path(filename).read_text(encoding='utf-8'))
    if data.get('request_id') != request_id or Path(data.get('frame', '')).resolve() != frame.resolve():
        raise ValueError('推理结果不属于本次相机请求')
    if data.get('frame_sha256') != hashlib.sha256(frame.read_bytes()).hexdigest():
        raise ValueError('推理结果与本次图像校验和不一致')
    with np.load(frame, allow_pickle=False) as image:
        timestamp = float(image['timestamp'])
    if not np.isfinite(data.get('timestamp', np.nan)) or abs(data['timestamp'] - timestamp) > 1e-6:
        raise ValueError('推理结果时间戳与采集帧不一致')
    return data


def run_worker(command, log, timeout, tick):
    """独立进程失败/超时即抛异常；退出时清理进程组，不回退到旧位姿。"""
    env = os.environ.copy()
    for name in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD'):
        env.pop(name, None)
    with Path(log).open('w', encoding='utf-8') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        try:
            deadline = time.monotonic() + timeout
            while process.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'FoundationPose 推理超过 {timeout:g} 秒，未执行动作')
                tick()
                time.sleep(.1)
            if process.returncode:
                raise RuntimeError(f'FoundationPose 退出码 {process.returncode}，详见 {log}')
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()


class VisualPipeline:
    """管理本次请求与阶段日志；原始图像、位姿、日志均保存在同一输出目录。"""

    def __init__(self, simulation, root, foundationpose_root, timeout):
        self.sim = simulation
        self.root = Path(root)
        self.repo = Path(foundationpose_root).resolve()
        self.timeout = timeout
        self.request_id = uuid.uuid4().hex
        self.folder = simulation.output / 'pipeline' / self.request_id
        self.folder.mkdir(parents=True)
        self.report = {'request_id': self.request_id, 'mask_source': 'simulator_semantic_ground_truth',
                       'same_simulation_world': True, 'online_visual_feedback': False,
                       'directory': str(self.folder), 'events': []}
        self.status('starting')

    def status(self, state, **details):
        """每个阶段持久化状态；中断也不会留下旧成功标记。"""
        self.report.update(status=state, **details)
        self.report['events'].append({'status': state, 'wall_time_unix': time.time(),
                                      'simulation_time_s': self.sim.sim_time})
        (self.sim.output / 'pipeline.json').write_text(json.dumps(self.report, indent=2), encoding='utf-8')

    def run(self):
        """先采集，再暂停物理等待推理；全部校验与 IK 成功后才允许运动。"""
        from .camera import HeadRGBDCamera
        # 复用独立的网格导出工具，只读取 CAD，不读取场景真值位姿。
        from scripts.export_pose_meshes import export_meshes
        wrapper = self.root / 'scripts/foundationpose_python.sh'
        if not (self.repo / 'estimater.py').is_file():
            raise FileNotFoundError(f'FoundationPose 未安装：{self.repo}')
        self.status('capturing')
        print('[视觉流程] 采集本次头部 RGB-D（使用仿真语义 mask）', flush=True)
        camera = HeadRGBDCamera(self.sim.config, debug_masks=True)
        try:
            for _ in range(120):
                self.sim.step()
            capture = camera.save(self.sim.stage, self.folder / 'camera')
        finally:
            camera.sensor.destroy()
        export_meshes(self.sim.config, self.folder / 'meshes')
        frame = capture / 'frame.npz'
        estimate = self.folder / 'estimated_pose.json'
        self.sim.world.pause()
        frozen_step = self.sim.world.current_time_step_index
        try:
            self.status('estimating', frame=str(frame), estimate=str(estimate))
            command = ['bash', str(wrapper), str(self.root / 'scripts/estimate_pose.py'),
                       '--foundationpose-root', str(self.repo), '--frame', str(frame),
                       '--meshes', str(self.folder / 'meshes'),
                       '--body-mask', str(capture / 'body_mask_gt.png'),
                       '--handle-mask', str(capture / 'handle_mask_gt.png'),
                       '--mask-source', 'simulator_semantic_ground_truth',
                       '--request-id', self.request_id, '--output', str(estimate)]
            def tick():
                if not self.sim.app.is_running():
                    raise RuntimeError('仿真应用关闭，取消推理')
                if self.sim.world.is_playing():
                    self.sim.world.pause()
                self.sim.app.update()
                if self.sim.world.current_time_step_index != frozen_step:
                    raise RuntimeError('推理期间物理状态发生变化，拒绝执行旧观测')
            run_worker(command, self.folder / 'inference.log', self.timeout, tick)
            self.status('validating')
            validate_response(estimate, frame, self.request_id)
            self.status('planning')
            self.sim.prepare_visual_plan(estimate)
            self.sim.perception_report.update(request_id=self.request_id, same_simulation_world=True,
                                              frame_sha256=hashlib.sha256(frame.read_bytes()).hexdigest())
        except BaseException:
            # 失败保持暂停，调用者保存失败记录并结束；不执行任何备用轨迹。
            raise
        self.sim.world.play()
        self.status('executing', frozen_physics_step=int(frozen_step))
