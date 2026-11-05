"""验证请求绑定、超时与子进程失败；不需要 GPU，不用旧位姿替代推理。"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from faucet_sim.pipeline import validate_response, run_worker


class PipelineTests(unittest.TestCase):
    """失败路径必须抛出异常，调用者不会进入执行阶段。"""

    def test_reject_stale_or_modified_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            frame, result = Path(folder) / 'frame.npz', Path(folder) / 'result.json'
            np.savez(frame, timestamp=1.25)
            data = {'request_id': 'current', 'frame': str(frame), 'timestamp': 1.25,
                    'frame_sha256': hashlib.sha256(frame.read_bytes()).hexdigest()}
            result.write_text(json.dumps(data))
            self.assertEqual(validate_response(result, frame, 'current')['request_id'], 'current')
            with self.assertRaises(ValueError):
                validate_response(result, frame, 'previous')
            data['timestamp'] = 2.
            result.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                validate_response(result, frame, 'current')
            np.savez(frame, timestamp=1.5)
            with self.assertRaises(ValueError):
                validate_response(result, frame, 'current')

    def test_worker_failure_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError, '7'):
                run_worker([sys.executable, '-c', 'raise SystemExit(7)'],
                           Path(folder) / 'worker.log', 5, lambda: None)

    def test_timeout_cleans_worker(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / 'late_output'
            command = [sys.executable, '-c',
                       'import time,pathlib,sys; time.sleep(5); pathlib.Path(sys.argv[1]).touch()', str(marker)]
            with self.assertRaises(TimeoutError):
                run_worker(command, Path(folder) / 'worker.log', .1, lambda: None)
            self.assertFalse(marker.exists())


if __name__ == '__main__':
    unittest.main()
