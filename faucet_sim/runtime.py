"""Isaac Sim 物理执行与验收。必须在创建 SimulationApp 后导入本模块。"""
import csv
import json
import time
from pathlib import Path
import numpy as np
from pxr import UsdPhysics, PhysxSchema, UsdGeom, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from .geometry import world_pose, pose_error
from .kinematics import RightArmKinematics, FaucetKinematics
from .planning import plan, save_plan
from .scene import build_scene
from .grasp import FrictionGrasp
from .visuals import SimulationDisplay
from .collision import ContactMonitor


class FaucetSimulation:
    """管理场景、规划、执行、测量和日志；机器人由位置驱动跟踪轨迹。"""

    def __init__(self, app, config, output, headless=False, realtime=False, capture_frames=False,
                 pose_estimate=None, camera_only=False):
        """app 为已启动应用；output 是该次运行独立输出目录。"""
        self.app, self.config = app, config
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.headless, self.realtime = headless, realtime
        self.capture_frames, self.capture_jobs = capture_frames, []
        # 可选录制观察者只读取状态和控制摄影机；普通仿真无需构造。
        self.observer = None
        source = build_scene(config, self.output / 'scene.usda')
        self.robot_model = RightArmKinematics(source, world_pose(source.GetPrimAtPath(
            '/World/Robot/arm_base_link')), config['tcp_in_gripper'])
        self.faucet_model = FaucetKinematics(source, world_pose(source.GetPrimAtPath(
            '/World/Faucet/body')), config)
        initial_valve, self.perception_report = None, None
        planning_faucet = self.faucet_model
        if pose_estimate:
            from .perception import apply_pose_estimate
            planning_faucet = FaucetKinematics(source, np.eye(4), config)
            initial_valve, self.perception_report = apply_pose_estimate(planning_faucet, pose_estimate)
            # 快照模式只支持当前默认的初始关闭状态；任意初始状态需在线观测。
            if np.max(np.abs(np.rad2deg(initial_valve))) > 3.:
                raise ValueError('快照执行模式要求初始阀角接近零位（3°以内）；非零初始状态请先做离线规划')
        self.segments, self.duration = [], 0.
        if not camera_only:
            print('[规划] 开始检查完整轨迹', flush=True)
            self.segments = plan(config, self.robot_model, planning_faucet, initial_valve=initial_valve)
            self.duration = save_plan(self.segments, self.output)
            print(f'[规划] {len(self.segments)} 个阶段，{self.duration:.2f} 秒', flush=True)
        omni.usd.get_context().open_stage(str(self.output / 'scene.usda'))
        self.stage = omni.usd.get_context().get_stage()
        physics = PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath('/World/PhysicsScene'))
        physics.CreateSolverTypeAttr('TGS')
        physics.CreateEnableGPUDynamicsAttr(False)
        physics.CreateBroadphaseTypeAttr('MBP')
        self.world = World(physics_dt=config['physics_dt'], rendering_dt=1/60,
                           stage_units_in_meters=1., backend='numpy', device='cpu')
        self.robot = self.world.scene.add(SingleArticulation('/World/Robot', name='dual_arm',
                                                            reset_xform_properties=False))
        self.valve = self.world.scene.add(SingleArticulation('/World/Faucet', name='faucet',
                                                            reset_xform_properties=False))
        self.contacts = ContactMonitor(self.world)
        self.world.reset()
        self.right_indices = np.array([self.robot.get_dof_index(name) for name in self.robot_model.names])
        self.finger_index = self.robot.get_dof_index('arm_r_joint_finger')
        self.valve_indices = np.array([self.valve.get_dof_index(name) for name in ['temperature_yaw', 'flow_lift']])
        self.robot.set_solver_position_iteration_count(128)
        self.robot.set_solver_velocity_iteration_count(4)
        self.valve.set_solver_position_iteration_count(128)
        self.valve.set_solver_velocity_iteration_count(4)
        n = self.robot.num_dof
        # 保持左臂零位；右臂只通过七轴目标和一个手指目标动作。
        self.command = np.zeros(n)
        self.command[self.right_indices] = np.deg2rad(config['home_right_deg'])
        self.command[self.finger_index] = np.deg2rad(config['finger_open_deg'])
        kps, kds = np.full(n, 12000.), np.full(n, 120.)
        efforts = np.array([130. if name.endswith('_1') else 70. if name.endswith(('_2', '_3', '_4')) else 12.
                            for name in self.robot.dof_names])
        # 活动手指使用柔顺的小力矩驱动，在把手表面受阻后停止，不能强压入模型。
        kps[self.finger_index], kds[self.finger_index] = config['finger_stiffness'], config['finger_damping']
        efforts[self.finger_index] = config['finger_max_effort']
        self.robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
        self.robot.get_articulation_controller().set_max_efforts(efforts)
        # 只在初始化时设置一次状态；正常运行始终使用动力学驱动，禁止逐帧瞬移。
        self.robot.set_joint_positions(self.command)
        self.robot.set_joint_velocities(np.zeros(n))
        self.valve.set_joint_positions(np.zeros(self.valve.num_dof))
        self.passive_valve()
        self.grasp = FrictionGrasp(self.stage, self.robot_model, self.faucet_model)
        self.grasp.assert_no_attachment()
        self.step_count = 0
        self.sim_time = 0.
        self.records, self.endpoints = [], []
        self.capture = None
        self.display = SimulationDisplay(self.stage, config, show_panel=not headless)
        set_camera_view(eye=np.array([1.2, 1.25, 1.25]), target=np.array([0., .20, .65]))
        for _ in range(90):
            self.step()
        print('[物理] 初始化完成，右臂关节：' + ', '.join(self.robot_model.names), flush=True)

    def passive_valve(self):
        """所有阶段关闭龙头位置刚度，只保留配置中的被动阻尼。"""
        self.valve.get_articulation_controller().set_gains(kps=np.zeros(self.valve.num_dof),
                                                         kds=np.full(self.valve.num_dof, self.config['faucet_joint_damping']))

    def prepare_visual_plan(self, filename):
        """在本次相机观测后规划，不重建世界、不重新初始化物理状态。"""
        from .perception import apply_pose_estimate
        model = FaucetKinematics(self.stage, np.eye(4), self.config)
        initial, report = apply_pose_estimate(model, filename)
        if np.max(np.abs(np.rad2deg(initial))) > 3.:
            raise ValueError('当前单帧流程要求初始关闭，估计阀角必须在 3°以内')
        segments = plan(self.config, self.robot_model, model, initial_valve=initial)
        self.duration = save_plan(segments, self.output)
        self.segments, self.perception_report = segments, report
        print(f'[视觉规划] {len(segments)} 个阶段，{self.duration:.2f} 秒', flush=True)

    def measured(self):
        """返回物理反馈：右臂七轴角度、龙头左右角和抬升角（弧度）。"""
        return (self.robot.get_joint_positions(self.right_indices),
                self.valve.get_joint_positions(self.valve_indices))

    def wait_for_play(self):
        """暂停时只刷新界面，保留当前轨迹采样；Stop 不等同于可恢复暂停。"""
        announced = False
        while True:
            if not self.app.is_running():
                raise RuntimeError('用户关闭了仿真窗口，流程提前结束')
            if self.world.is_stopped():
                raise RuntimeError('用户停止了仿真；中途挂起请使用 Pause，恢复请使用 Play')
            if self.world.is_playing():
                if announced:
                    print('[仿真] 已恢复，从暂停处继续', flush=True)
                return
            if not announced:
                print('[仿真] 已暂停，点击 Play 继续', flush=True)
                announced = True
            self.world.render()
            time.sleep(.01)

    def step(self):
        """仅在物理实际前进后消费一个采样；暂停期间不计时、不采集接触。"""
        while True:
            self.wait_for_play()
            self.robot.apply_action(ArticulationAction(joint_positions=self.command))
            start = time.monotonic()
            self.contacts.begin_step()
            before = self.world.current_time_step_index
            self.world.step(render=(self.step_count % 3 == 0))
            # 界面刷新可能恰好处理了 Pause；没有物理步就等待恢复并重试同一目标。
            if self.world.current_time_step_index > before:
                break
        self.contacts.sample()
        self.step_count += 1
        self.sim_time += self.config['physics_dt']
        if self.observer is not None:
            self.observer.after_step(self)
        if self.realtime:
            time.sleep(max(0., self.config['physics_dt'] - (time.monotonic() - start)))
        # 若暂停发生在已经完成物理步的渲染帧，先等待，避免推进到下一阶段验收。
        self.wait_for_play()

    def record(self, segment, index):
        """记录实际关节、TCP 跟踪误差及目标阀角；实际值来自 PhysX。"""
        right, valve = self.measured()
        if not np.isfinite(np.r_[right, valve]).all():
            raise RuntimeError('物理状态包含 NaN/Inf')
        position, rotation = pose_error(self.robot_model.forward(right), segment.target[index])
        bilateral = bool(self.contacts.last.get('fixed_contact_points') and
                         self.contacts.last.get('moving_contact_points'))
        row = {'time_s': self.sim_time, 'phase': segment.name,
               'yaw_target_deg': float(np.rad2deg(segment.faucet[index, 0])),
               'lift_target_deg': float(np.rad2deg(segment.faucet[index, 1])),
               'yaw_actual_deg': float(np.rad2deg(valve[0])), 'lift_actual_deg': float(np.rad2deg(valve[1])),
               'tcp_error_mm': position*1000., 'rotation_error_deg': float(np.rad2deg(rotation)),
               'grasp_active': bool(self.grasp.active and bilateral),
               'grasp_attempt_active': self.grasp.active,
               'finger_actual_deg': float(np.rad2deg(self.robot.get_joint_positions([self.finger_index])[0]))}
        row.update({f'q{i+1}_rad': float(value) for i, value in enumerate(right)})
        row.update(self.display.update(*valve, segment.kind, row['grasp_active']))
        row.update(self.contacts.last)
        row.update(self.grasp.observe(right, valve))
        self.records.append(row)
        # 未完成夹持前出现毫米级阻塞时立即停止，避免继续接近把把手顶开。
        if segment.kind in ('approach', 'grasp') and (position > .005 or rotation > np.deg2rad(5)):
            raise RuntimeError(f'{segment.name} 接近受阻：TCP 偏差 {position*1000:.1f} mm / '
                               f'{np.rad2deg(rotation):.1f}°，已停止，未进入开阀阶段')
        # 物理执行偏离规划时停止，避免把明显失败的动画当成完成。
        if position > .05 or rotation > np.deg2rad(20):
            raise RuntimeError(f'{segment.name} 跟踪误差过大：{position*1000:.1f} mm / {np.rad2deg(rotation):.1f}°')

    def execute(self):
        """执行完整流程；每次操作结束测量阀角误差并验收。"""
        if not self.segments:
            raise RuntimeError('没有已验证的轨迹，拒绝执行')
        for segment_index, segment in enumerate(self.segments):
            print(f'[阶段] {segment.name}', flush=True)
            if self.observer is not None:
                self.observer.phase_start(self, segment, segment_index)
            if segment.kind == 'operate' and not self.grasp.active:
                right, valve = self.measured()
                finger = self.robot.get_joint_positions([self.finger_index])[0]
                # 物体会阻挡手指达到空载闭合角，因此验收两侧真实接触，不强迫角度归零。
                if finger >= np.deg2rad(self.config['finger_open_deg'] - 1):
                    raise RuntimeError('活动手指尚未完成闭合动作')
                if not self.contacts.last.get('fixed_contact_points') or not self.contacts.last.get('moving_contact_points'):
                    raise RuntimeError('抓取失败：上下两指尚未同时接触把手')
                self.capture = self.grasp.begin(right, valve)
                self.capture['contact_points'] = self.contacts.last.get('contact_points', 0)
                self.capture['finger_actual_deg'] = float(np.rad2deg(finger))
                self.passive_valve()
            if segment.kind == 'release':
                # 不锁定末态；松爪后按重力、接触与阻尼运动。
                self.grasp.release()
            for index in range(len(segment.time)):
                self.command[self.right_indices] = segment.q[index]
                self.command[self.finger_index] = segment.finger[index]
                self.step()
                if index % 6 == 0 or index == len(segment.time) - 1:
                    self.record(segment, index)
            # 固定目标后等待有限时间，让动力学跟踪器收敛。
            for _ in range(round(.5 / self.config['physics_dt'])):
                self.step()
            self.record(segment, -1)
            if segment.kind == 'operate':
                _, measured = self.measured()
                error = np.rad2deg(measured - segment.faucet[-1])
                endpoint = {'phase': segment.name, 'target_deg': np.rad2deg(segment.faucet[-1]).tolist(),
                            'actual_deg': np.rad2deg(measured).tolist(), 'error_deg': error.tolist(),
                            'passed': bool(np.max(np.abs(error)) < 2.)}
                self.endpoints.append(endpoint)
                print('[验收] ' + json.dumps(endpoint, ensure_ascii=False), flush=True)
                if not endpoint['passed']:
                    raise RuntimeError(f'{segment.name} 实际阀角偏差超过 2°：{error}')
            if self.capture_frames:
                self.capture_frame(f'{segment_index+1:02d}_{segment.kind}.png')
                if segment.kind == 'operate' and segment_index == 6:
                    # 额外保留一张抓取局部视图，便于确认手指和把手的相对位置。
                    self.capture_grasp_detail('grasp_detail.png')
                if segment.kind == 'grasp':
                    self.capture_grasp_detail('grasp_closed_detail.png')
        actual, valve = self.measured()
        final_error = np.rad2deg(actual - self.segments[-1].q[-1])
        if np.max(np.abs(final_error)) > 2.:
            raise RuntimeError('右臂未返回初始位置')
        return {'passed': True, 'mode': 'physx_friction_only', 'capture': self.capture,
                'endpoints': self.endpoints, 'physics_steps': self.step_count,
                'final_right_error_deg': final_error.tolist(), 'final_faucet_deg': np.rad2deg(valve).tolist(),
                'simulation_time_s': self.sim_time,
                'collision_check': {'enabled': True, 'max_penetration_mm': self.contacts.max_penetration * 1000.,
                                    'handle_contact_steps': self.contacts.handle_contact_steps,
                                    'contact_pairs': [list(pair) for pair in sorted(self.contacts.pairs)]},
                'max_tcp_error_mm': max(row['tcp_error_mm'] for row in self.records),
                'max_rotation_error_deg': max(row['rotation_error_deg'] for row in self.records)}

    def capture_frame(self, filename):
        """用 Isaac 自带视口截图 API 保存关键帧，不额外安装图像或录屏包。"""
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        folder = self.output / 'frames'
        folder.mkdir(exist_ok=True)
        # 首先刷新实际末帧；world.render 只渲染，不推进一个显式物理步。
        for _ in range(4):
            self.world.render()
        self.capture_jobs.append(capture_viewport_to_file(get_active_viewport(), str(folder / filename)))
        for _ in range(8):
            self.world.render()

    def capture_grasp_detail(self, filename):
        """对准实际夹爪拍局部图，便于检查未完成阶段的接触/碰撞。"""
        from omni.kit.viewport.utility import get_active_viewport
        point = self.robot_model.forward(self.measured()[0])[:3, 3]
        eye = point + np.array([.24, .06, .12])
        camera = UsdGeom.Camera.Define(self.stage, '/World/GraspCamera')
        camera.CreateFocalLengthAttr(30.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.005, 100.))
        camera.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*point), Gf.Vec3d(0, 0, 1)).GetInverse())
        viewport = get_active_viewport()
        previous = viewport.camera_path
        viewport.camera_path = camera.GetPath()
        self.capture_frame(filename)
        viewport.camera_path = previous

    def save(self, result):
        """成功或失败都保存 JSON 和 CSV，方便复查实际运行结果。"""
        if self.perception_report is not None:
            result['perception'] = dict(self.perception_report,
                mode='snapshot_initialization', ground_truth_evaluation=True,
                online_visual_feedback=False)
        if self.records:
            with (self.output / 'execution.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.records[0]))
                writer.writeheader()
                writer.writerows(self.records)
        result.setdefault('endpoints', self.endpoints)
        stiffness, damping = self.valve.get_articulation_controller().get_gains()
        result.update({'mode': 'physx_friction_only', 'capture': self.capture,
                       'grasp_audit': {'robot_faucet_attachment_joints': self.grasp.assert_no_attachment(),
                                       'faucet_drive_stiffness': np.asarray(stiffness).tolist(),
                                       'faucet_drive_damping': np.asarray(damping).tolist(),
                                       'faucet_commands_after_initial_state': 0},
                       'friction_parameters': {name: self.config[name] for name in (
                           'contact_static_friction', 'contact_dynamic_friction', 'finger_stiffness',
                           'finger_damping', 'finger_max_effort', 'faucet_joint_damping')},
                       'slip': {'max_relative_translation_mm': self.grasp.max_slip_mm,
                                'max_relative_rotation_deg': self.grasp.max_slip_deg},
                       'contact_forces': {'max_normal_force_n': self.contacts.max_normal_force,
                                          'max_friction_force_n': self.contacts.max_friction_force,
                                          'total_friction_impulse_ns': self.contacts.total_friction_impulse}})
        result.setdefault('collision_check', {'enabled': True,
                          'max_penetration_mm': self.contacts.max_penetration * 1000.,
                          'handle_contact_steps': self.contacts.handle_contact_steps,
                          'contact_pairs': [list(pair) for pair in sorted(self.contacts.pairs)]})
        (self.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
