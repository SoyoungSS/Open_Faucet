"""不启动 Isaac 界面的暂停回归：直接测试运行时方法，模拟时间轴和物理计数。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


def runtime_methods():
    """只加载控制循环方法，避免离线测试导入需要 SimulationApp 的模块。"""
    path = Path(__file__).resolve().parents[1] / 'faucet_sim/runtime.py'
    tree = ast.parse(path.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in ('step', 'wait_for_play')]
    namespace = {'time': SimpleNamespace(monotonic=lambda: 0., sleep=Mock()),
                 'ArticulationAction': lambda **kw: kw}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['FaucetSimulation']


class PauseTests(unittest.TestCase):
    """覆盖进入前暂停、渲染中暂停，以及停止和关闭窗口。"""

    def make_sim(self, paused=False, pause_during_step=False, advance=True):
        runtime = runtime_methods()
        sim = runtime.__new__(runtime)
        world = SimpleNamespace(current_time_step_index=0, playing=not paused)
        world.is_playing = lambda: world.playing
        world.is_stopped = Mock(return_value=False)
        sim.app = SimpleNamespace(is_running=Mock(return_value=True))
        sim.command = [1.]
        sim.robot = SimpleNamespace(apply_action=Mock())
        sim.contacts = SimpleNamespace(begin_step=Mock(), sample=Mock())
        sim.step_count, sim.sim_time = 0, 0.
        sim.config, sim.realtime = {'physics_dt': .01}, False
        sim.observer = SimpleNamespace(after_step=Mock())
        sim.world = world

        def render():
            self.assertEqual(sim.step_count, world.current_time_step_index)
            world.playing = True

        def step(**kwargs):
            if pause_during_step and not world.render.called:
                world.playing = False
                if not advance:
                    return
            world.current_time_step_index += 1

        world.render, world.step = Mock(side_effect=render), Mock(side_effect=step)
        return sim

    def test_paused_before_step(self):
        sim = self.make_sim(paused=True)
        sim.step()
        sim.world.render.assert_called_once()
        self.assertEqual((sim.step_count, sim.sim_time), (1, .01))
        sim.contacts.sample.assert_called_once()
        sim.observer.after_step.assert_called_once_with(sim)

    def test_pause_in_render_without_physics_retries_same_command(self):
        sim = self.make_sim(pause_during_step=True, advance=False)
        sim.step()
        self.assertEqual(sim.world.step.call_count, 2)
        self.assertEqual(sim.robot.apply_action.call_args_list[0], sim.robot.apply_action.call_args_list[1])
        self.assertEqual((sim.step_count, sim.sim_time), (1, .01))
        sim.contacts.sample.assert_called_once()

    def test_pause_after_physics_does_not_repeat_step(self):
        sim = self.make_sim(pause_during_step=True)
        sim.step()
        sim.world.step.assert_called_once()
        sim.world.render.assert_called_once()
        self.assertEqual(sim.step_count, 1)

    def test_stop_and_close_exit_wait(self):
        for closed in (False, True):
            with self.subTest(closed=closed):
                sim = self.make_sim(paused=True)
                sim.app.is_running.return_value = not closed
                sim.world.is_stopped.return_value = not closed
                with self.assertRaisesRegex(RuntimeError, '关闭|停止'):
                    sim.step()
                sim.robot.apply_action.assert_not_called()
                self.assertEqual(sim.step_count, 0)
