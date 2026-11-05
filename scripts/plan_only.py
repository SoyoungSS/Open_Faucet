"""无需启动 Isaac Sim 的资产解析和规划检查；使用同一套正式规划代码。"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from faucet_sim.config import load_config
from faucet_sim.scene import build_scene
from faucet_sim.geometry import world_pose
from faucet_sim.kinematics import RightArmKinematics, FaucetKinematics
from faucet_sim.planning import plan, save_plan


def main():
    """读取命令行，输出场景、轨迹及规划摘要；不可达时非零退出。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--output', default='outputs/plan')
    parser.add_argument('--pose-estimate', help='FoundationPose 位姿 JSON，用于视觉规划验证')
    args = parser.parse_args()
    config = load_config(args.config)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    stage = build_scene(config, output / 'scene.usda')
    robot = RightArmKinematics(stage, world_pose(stage.GetPrimAtPath('/World/Robot/arm_base_link')),
                               config['tcp_in_gripper'])
    faucet = FaucetKinematics(stage, world_pose(stage.GetPrimAtPath('/World/Faucet/body')), config)
    initial_valve = None
    if args.pose_estimate:
        import json
        from faucet_sim.perception import apply_pose_estimate
        initial_valve, report = apply_pose_estimate(faucet, args.pose_estimate)
        (output / 'perception.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    segments = plan(config, robot, faucet, initial_valve=initial_valve)
    duration = save_plan(segments, output)
    print(f'规划完成：{len(segments)} 个阶段，仿真时间 {duration:.2f} 秒；输出 {output}')


if __name__ == '__main__':
    main()
