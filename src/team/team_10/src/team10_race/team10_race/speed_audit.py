"""Optional bounded, asynchronous validation trace; never a driving input."""
import atexit
import json
import os
from pathlib import Path
import queue
import threading
import time


class SpeedAudit:
    def __init__(self, name):
        self.enabled = bool(os.environ.get('HANLA_SPEED_AUDIT_DIR'))
        self.dropped = 0
        self.thread = None
        if not self.enabled:
            return
        folder = Path(os.environ['HANLA_SPEED_AUDIT_DIR'])
        folder.mkdir(parents=True, exist_ok=True)
        self.file = (folder / (name + '.jsonl')).open('x', buffering=1024*1024)
        self.queue = queue.Queue(maxsize=4096)
        self.closed = False
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()
        atexit.register(self.close)

    def emit(self, **row):
        if not self.enabled or self.closed:
            return
        row.update(wall=time.time(), mono=time.monotonic())
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        last = time.monotonic()
        while True:
            try:
                row = self.queue.get(timeout=.25)
            except queue.Empty:
                self.file.flush()
                continue
            if row is None:
                break
            self.file.write(json.dumps(row, separators=(',', ':')) + '\n')
            if time.monotonic() - last > .5:
                self.file.flush()
                last = time.monotonic()
        self.file.write(json.dumps(dict(event='audit_end', dropped=self.dropped))+'\n')
        self.file.close()

    def close(self):
        if self.thread and not self.closed:
            self.closed = True
            self.queue.put(None)
            self.thread.join(timeout=5)
