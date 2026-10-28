import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('feedback', Path(__file__).resolve().parents[1] / 'live_feedback_audit.py')
feedback = importlib.util.module_from_spec(spec)
spec.loader.exec_module(feedback)

class FeedbackAudit(unittest.TestCase):
    def run_archive(self, rows, mover_rows, results='mode: live-demo\nend_s: 10\n'):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'run.tar.gz'
            files = {'run/mission/results.yaml': results,
                     'run/mission/raw/scene-agent0-run/raw/groundtruth_body.tum': rows,
                     'run/mover_agent_0.jsonl': ''.join(json.dumps(x)+'\n' for x in mover_rows)}
            with tarfile.open(path, 'w:gz') as t:
                for name, text in files.items():
                    data = text.encode(); m = tarfile.TarInfo(name); m.size = len(data)
                    t.addfile(m, io.BytesIO(data))
            return feedback.audit(path)['agents']['0']

    def test_motion_and_commands_after_horizon_are_excluded(self):
        r = self.run_archive('0 0 0 0 0 0 0 1\n5 3 4 0 0 0 0 1\n11 30 40 0 0 0 0 1\n',
                             [{'cmd_stamp_ns': 1_000_000_000, 'applied_ns': 2_000_000_000},
                              {'cmd_stamp_ns': 2_000_000_000, 'applied_ns': 11_000_000_000}])
        self.assertEqual(r['gt_translation_path_m'], 5)
        self.assertEqual(r['command_age_sim_s']['max'], 1)
        self.assertEqual(r['applied_commands_after_horizon'], 1)

    def test_missing_gt_is_not_stationary_evidence(self):
        r = self.run_archive('', [])
        self.assertIsNone(r['gt_translation_path_m'])
        self.assertIsNone(r['command_age_sim_s']['max'])

    def test_stationary_and_future_stamp_are_reported(self):
        r = self.run_archive('0 1 2 0 0 0 0 1\n5 1 2 0 0 0 0 1\n',
                             [{'cmd_stamp_ns': 3_000_000_000, 'applied_ns': 2_000_000_000}])
        self.assertEqual(r['gt_translation_path_m'], 0)
        self.assertEqual(r['negative_command_age_count'], 1)

    def test_missing_live_result_rejected(self):
        with self.assertRaises(ValueError):
            self.run_archive('', [], 'mode: replay\nend_s: 10\n')

if __name__ == '__main__':
    unittest.main()
