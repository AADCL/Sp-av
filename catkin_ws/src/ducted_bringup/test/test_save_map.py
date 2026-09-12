import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch


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
            (output / 'mapping_metadata.yaml').write_text('format_version: 1\n')
            (output / 'observed_occupancy.pcd').write_text('data')
            return True
        self.assertEqual(self.run_save(save), 0)
        self.assertEqual(self.status()['state'], 'SUCCEEDED')

    def test_static_only_bundle_is_complete_when_metadata_explicitly_omits_grid(self):
        def save():
            output = self.root / 'map'
            output.mkdir()
            for name in self.mod.BUNDLE:
                if name != 'observed_occupancy.pcd':
                    (output / name).write_text('data')
            (output / 'mapping_metadata.yaml').write_text('format_version: 2\noccupancy_exported: false\n')
            return True
        self.assertEqual(self.run_save(save), 0)
        self.assertEqual(self.status()['state'], 'SUCCEEDED')

    def test_requested_occupancy_cannot_be_missing(self):
        def save():
            output = self.root / 'map'
            output.mkdir()
            for name in self.mod.BUNDLE:
                if name != 'observed_occupancy.pcd':
                    (output / name).write_text('data')
            (output / 'mapping_metadata.yaml').write_text('format_version: 2\noccupancy_exported: true\n')
            return True
        self.assertEqual(self.run_save(save), 1)
        self.assertIn('observed_occupancy.pcd', self.status()['message'])

    def test_malformed_manifest_cannot_claim_success(self):
        def save():
            output = self.root / 'map'
            output.mkdir()
            for name in self.mod.BUNDLE:
                (output / name).write_text('data')
            return True
        self.assertEqual(self.run_save(save), 1)

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

    def test_wait_reports_complete_map_destination(self):
        def advance(_interval):
            self.mod.write_status(self.root / 'status.json',
                                  dict(state='SUCCEEDED', destination='/maps/site_a'))
        self.mod.write_status(self.root / 'status.json', dict(state='SAVING'))
        with patch.object(self.mod.time, 'sleep', advance), patch('builtins.print') as output:
            self.assertEqual(self.mod.wait_for_save(self.root, interval=.01), 0)
        self.assertIn(str(Path('/maps/site_a') / 'GlobalMap.pcd'),
                      '\n'.join(call.args[0] for call in output.call_args_list))

    def test_wait_rejected_save_has_nonzero_exit(self):
        self.mod.write_status(self.root / 'status.json', dict(state='FAILED', message='rejected'))
        with patch('builtins.print'):
            self.assertEqual(self.mod.wait_for_save(self.root), 1)

    def test_exited_worker_cannot_look_like_an_active_save(self):
        self.mod.write_status(self.root / 'status.json', dict(state='SAVING'))
        self.mod.write_status(self.root / 'process.json', dict(pid=2147483647, start_ticks='0'))
        with patch('builtins.print'):
            self.assertEqual(self.mod.wait_for_save(self.root), 2)

    def test_interrupting_progress_does_not_change_background_job(self):
        self.mod.write_status(self.root / 'status.json', dict(state='SAVING'))
        with patch.object(self.mod.time, 'sleep', side_effect=KeyboardInterrupt), patch('builtins.print'):
            self.assertEqual(self.mod.wait_for_save(self.root), 130)
        self.assertEqual(self.status()['state'], 'SAVING')

    def test_worker_finishing_during_status_read_is_not_unknown(self):
        self.mod.write_status(self.root / 'status.json', dict(state='SAVING'))
        self.mod.write_status(self.root / 'process.json', dict(pid=2147483647, start_ticks='42'))
        read_text = Path.read_text
        def finish(path, *args, **kwargs):
            if path.name == 'stat':
                self.mod.write_status(self.root / 'status.json',
                                      dict(state='SUCCEEDED', destination='/maps/finished'))
                raise FileNotFoundError()
            return read_text(path, *args, **kwargs)
        with patch.object(Path, 'read_text', finish):
            self.assertEqual(self.mod.read_status(self.root)['state'], 'SUCCEEDED')

    def test_zombie_worker_is_no_longer_saving(self):
        self.mod.write_status(self.root / 'status.json', dict(state='SAVING'))
        self.mod.write_status(self.root / 'process.json', dict(pid=2147483647, start_ticks='42'))
        read_text = Path.read_text
        def zombie(path, *args, **kwargs):
            if path.name == 'stat':
                return '2147483647 (python3) Z ' + '0 ' * 18 + '42'
            return read_text(path, *args, **kwargs)
        with patch.object(Path, 'read_text', zombie):
            self.assertEqual(self.mod.read_status(self.root)['state'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
