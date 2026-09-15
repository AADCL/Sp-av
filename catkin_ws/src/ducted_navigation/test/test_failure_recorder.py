import io
import json
import pathlib
import sys
import tempfile
import threading
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from ducted_navigation.failure_recorder import FailureRecorder


class Message:
    def __init__(self, data): self.data = data
    def serialize(self, stream): stream.write(self.data)


class FailureRecorderTest(unittest.TestCase):
    def test_request_bytes_and_goal_identity_are_preserved(self):
        with tempfile.TemporaryDirectory() as root:
            recorder = FailureRecorder(root, slots=2)
            recorder.submit(Message(b'exact ROS request'), {'request_id': 'run-1', 'reason': 'occupied'})
            recorder.close(2)
            metadata = json.loads((pathlib.Path(root)/'failure_0.json').read_text())
            self.assertEqual(metadata['request_id'], 'run-1')
            self.assertEqual((pathlib.Path(root)/'failure_0.bin').read_bytes(), b'exact ROS request')

    def test_blocked_disk_worker_does_not_block_submit_and_queue_keeps_latest(self):
        began, release = threading.Event(), threading.Event()
        class Slow(Message):
            def serialize(self, stream):
                began.set(); release.wait(2); super().serialize(stream)
        with tempfile.TemporaryDirectory() as root:
            recorder = FailureRecorder(root, slots=2)
            recorder.submit(Slow(b'first'), {'request_id': 'first'})
            self.assertTrue(began.wait(1))
            recorder.submit(Message(b'superseded'), {'request_id': 'old'})
            recorder.submit(Message(b'latest'), {'request_id': 'new'})
            release.set(); recorder.close(2)
            self.assertEqual((pathlib.Path(root)/'failure_1.bin').read_bytes(), b'latest')
            self.assertEqual(len(list(pathlib.Path(root).glob('*.bin'))), 2)


if __name__ == '__main__': unittest.main()
