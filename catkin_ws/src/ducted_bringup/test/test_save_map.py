import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/save_map.py'


class SaveMapTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('save_map_script', SCRIPT)
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def run_save(self, call, progress=lambda: {}):
        return self.mod.perform_save(self.root / 'map', self.root / 'status.json', call, progress, .01)

    def status(self):
        return json.loads((self.root / 'status.json').read_text())

    def test_success_requires_service_and_complete_bundle(self):
        def save():
            output = self.root / 'map'
            output.mkdir()
            for name in self.mod.BUNDLE:
                (output / name).write_text('data')
            return True
        self.assertEqual(self.run_save(save), 0)
        self.assertEqual(self.status()['state'], 'SUCCEEDED')

    def test_incomplete_bundle_is_failure(self):
        self.assertEqual(self.run_save(lambda: True), 1)
        self.assertEqual(self.status()['state'], 'FAILED')

    def test_rejected_save_is_failure(self):
        self.assertEqual(self.run_save(lambda: False), 1)
        self.assertEqual(self.status()['state'], 'FAILED')

    def test_connection_loss_is_unknown_not_retry(self):
        calls = []
        def fail():
            calls.append(1)
            raise RuntimeError('connection lost')
        self.assertEqual(self.run_save(fail), 2)
        self.assertEqual(self.status()['state'], 'UNKNOWN')
        self.assertEqual(len(calls), 1)

    def test_pending_is_visible_and_other_destination_progress_ignored(self):
        release = threading.Event()
        seen = threading.Event()
        def progress():
            seen.set()
            return {'destination': '/another/map', 'completed': 9, 'total': 10}
        worker = threading.Thread(target=lambda: self.run_save(lambda: release.wait(2) and False, progress))
        worker.start()
        try:
            self.assertTrue(seen.wait(1))
            status = self.status()
            self.assertEqual(status['state'], 'SAVING')
            self.assertNotIn('progress', status)
        finally:
            release.set()
            worker.join(3)


if __name__ == '__main__':
    unittest.main()
