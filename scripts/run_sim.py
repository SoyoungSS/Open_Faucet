"""启动右臂操作水龙头。先创建 SimulationApp，再导入任何仿真 API。"""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
from datetime import datetime


def main():
    """解析启动参数；失败返回非零退出码，窗口默认在任务结束后保留。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='无窗口运行物理验收')
    parser.add_argument('--quit-on-complete', action='store_true', help='流程结束后退出')
    parser.add_argument('--realtime', action='store_true', help='按仿真时间限速，方便观察')
    parser.add_argument('--config', help='自定义 JSON 配置文件')
    parser.add_argument('--output', help='输出目录，默认使用时间戳')
    parser.add_argument('--capture-frames', action='store_true', help='保存各阶段的实际渲染截图')
    parser.add_argument('--head-camera-only', action='store_true', help='仅导出头部较高相机 RGB-D，不执行开关动作')
    parser.add_argument('--pose-estimate', help='以 FoundationPose JSON 初始化规划；当前仅支持初始关闭状态')
    parser.add_argument('--debug-pose-masks', action='store_true', help='相机快照同时导出仿真真值 mask，仅用于接入调试')
    parser.add_argument('--visual-pipeline', action='store_true', help='同次仿真内采集、FoundationPose 估计、规划并执行（真值 mask）')
    parser.add_argument('--foundationpose-root', help='FoundationPose 代码目录，默认使用本机项目专用安装')
    parser.add_argument('--pose-timeout', type=float, default=300., help='独立推理超时秒数，默认 300')
    args = parser.parse_args()
    if args.head_camera_only and args.pose_estimate:
        parser.error('--head-camera-only 与 --pose-estimate 不能同时使用')
    if args.visual_pipeline and (args.head_camera_only or args.pose_estimate):
        parser.error('--visual-pipeline 不能与快照导出或旧位姿文件模式混用')
    if not 0 < args.pose_timeout < float('inf'):
        parser.error('--pose-timeout 必须是正有限值')
    if args.debug_pose_masks and not (args.head_camera_only or args.visual_pipeline):
        parser.error('--debug-pose-masks 需要相机采集模式')
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    output = Path(args.output or root / 'outputs' / datetime.now().strftime('%Y%m%d_%H%M%S')).resolve()
    output.mkdir(parents=True, exist_ok=True)
    # 覆盖指定目录重跑时先使旧成功标记失效，避免初始化失败仍留下旧 passed=true。
    result_path = output / 'result.json'
    result_path.write_text(json.dumps({'passed': False, 'status': 'starting'}), encoding='utf-8')
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 900,
                         'anti_aliasing': 0, 'renderer': 'RaytracedLighting'})
    simulation, pipeline, code = None, None, 0
    try:
        from faucet_sim.config import load_config
        from faucet_sim.runtime import FaucetSimulation
        config = load_config(args.config)
        (output / 'effective_config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
        simulation = FaucetSimulation(app, config, output, args.headless, args.realtime, args.capture_frames,
                                      pose_estimate=args.pose_estimate,
                                      camera_only=args.head_camera_only or args.visual_pipeline)
        if args.visual_pipeline:
            from faucet_sim.pipeline import VisualPipeline
            pipeline = VisualPipeline(simulation, root, args.foundationpose_root or
                root / 'outputs/foundationpose_install/FoundationPose', args.pose_timeout)
            pipeline.run()
        if args.head_camera_only:
            from faucet_sim.camera import HeadRGBDCamera
            sensor = HeadRGBDCamera(config, debug_masks=args.debug_pose_masks)
            for _ in range(120):
                simulation.step()
            folder = sensor.save(simulation.stage, output / 'head_camera')
            result_path.write_text(json.dumps({'passed': False, 'status': 'camera_capture_only',
                                               'capture_directory': str(folder)}, indent=2), encoding='utf-8')
            print(f'[完成] 头部 RGB-D 已保存：{folder}', flush=True)
        else:
            result = simulation.execute()
            if pipeline is not None:
                pipeline.status('complete')
                result['pipeline'] = pipeline.report
            simulation.save(result)
            print(f'[完成] 验收通过，结果目录：{simulation.output}', flush=True)
        if not args.headless and not args.quit_on_complete:
            while app.is_running():
                simulation.step()
    except Exception as error:
        code = 1
        traceback.print_exc()
        if pipeline is not None:
            pipeline.status('failed', error=str(error))
        if simulation is not None:
            failure = {'passed': False, 'error': str(error)}
            if pipeline is not None:
                failure['pipeline'] = pipeline.report
            simulation.save(failure)
            if args.capture_frames:
                simulation.capture_frame('failure.png')
                simulation.capture_grasp_detail('failure_detail.png')
        else:
            result_path.write_text(json.dumps({'passed': False, 'error': str(error)}, ensure_ascii=False, indent=2),
                                   encoding='utf-8')
    finally:
        if simulation is not None:
            simulation.world.stop()
        if code:
            # Kit 快速关闭会直接结束进程，必须先将失败退出码传给 Kit。
            import omni.kit.app
            omni.kit.app.get_app().post_uncancellable_quit(code)
            app.update()
        app.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
