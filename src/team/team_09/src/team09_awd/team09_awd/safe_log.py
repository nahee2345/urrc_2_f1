"""Diagnostic disk errors must not crash the driving callback."""
class BestEffortLog:
    def __init__(self,path):
        self.name=str(path);self.stream=None;self.failed=False
        try:self.stream=path.open('w',newline='')
        except OSError:self.failed=True
    def write(self,data):
        if self.stream is not None and not self.failed:
            try:return self.stream.write(data)
            except OSError:self.failed=True
        return len(data)
    def flush(self):
        if self.stream is not None and not self.failed:
            try:self.stream.flush()
            except OSError:self.failed=True
    def close(self):
        if self.stream is not None:
            try:self.stream.close()
            except OSError:self.failed=True
