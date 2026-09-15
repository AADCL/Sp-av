"""Bounded asynchronous evidence for failed read-only planning requests."""
import hashlib
import io
import json
import os
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Event, Thread


class FailureRecorder:
    def __init__(self, directory, slots=8, on_error=None):
        self.directory = Path(directory).expanduser()
        self.slots = int(slots)
        if not 1 <= self.slots <= 32:
            raise ValueError('failure recorder slots must be between 1 and 32')
        self.on_error = on_error or (lambda error: None)
        self.pending, self.stop = Queue(maxsize=1), Event()
        self.worker = Thread(target=self._run, name='ego-failure-recorder', daemon=True)
        self.worker.start()

    def submit(self, request, metadata):
        if self.stop.is_set(): return
        item = (request, dict(metadata))
        try:
            self.pending.put_nowait(item)
        except Full:
            try: self.pending.get_nowait()
            except Empty: pass
            try: self.pending.put_nowait(item)
            except Full: pass

    def _run(self):
        index = 0
        while not self.stop.is_set() or not self.pending.empty():
            try: request, metadata = self.pending.get(timeout=.1)
            except Empty: continue
            try:
                buffer = io.BytesIO(); request.serialize(buffer)
                payload = buffer.getvalue()
                if len(payload) > 4 * 1024 * 1024:
                    raise ValueError('failure snapshot exceeds 4 MiB limit')
                metadata['sha256'] = hashlib.sha256(payload).hexdigest()
                metadata['format'] = 'ducted_planning/PlanLocalRequest ROS serialization'
                self.directory.mkdir(parents=True, exist_ok=True)
                stem = self.directory / ('failure_%d' % (index % self.slots))
                for suffix, content in (('.bin', payload), ('.json', json.dumps(metadata, indent=2).encode())):
                    temporary = Path(str(stem)+suffix+'.tmp')
                    temporary.write_bytes(content)
                    os.replace(str(temporary), str(stem)+suffix)
                index += 1
            except Exception as error:
                self.on_error(str(error))

    def close(self, timeout=.2):
        self.stop.set()
        self.worker.join(timeout)
