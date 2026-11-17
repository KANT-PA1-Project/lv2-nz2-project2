"""Track launcher-owned Linux sessions with pidfds, including surviving descendants.

Register immediately after Popen(start_new_session=True), refresh periodically,
and stop before removing shared frame files. An empty tracked session is never
rediscovered by its numeric ID: that ID may later belong to another process.
"""
import os
from pathlib import Path
import select
import signal
import time


def _identity(pid):
    proc = Path('/proc') / str(pid)
    try:
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        return (proc.stat().st_uid, int(fields[3]), int(fields[19]))
    except (OSError, ValueError, IndexError):
        return None


class OwnedSession:
    def __init__(self, process):
        self.process = process
        self.sid = process.pid
        self.members = {}
        identity = _identity(process.pid)
        if identity is None or identity[:2] != (os.getuid(), self.sid):
            raise RuntimeError('Cannot verify new owned session leader')
        self._remember(process.pid, identity)
        if process.pid not in self.members:
            raise RuntimeError('Owned session leader exited before registration')
        self.refresh()

    def _remember(self, pid, identity):
        try:
            fd = os.pidfd_open(pid)
        except ProcessLookupError:
            return
        if _identity(pid) != identity:
            os.close(fd)
            return
        self.members[pid] = (fd, identity)

    def live(self):
        return {pid: fd for pid, (fd, _) in self.members.items()
                if not select.select([fd], [], [], 0)[0]}

    def refresh(self):
        # A live, previously verified member anchors this session's identity.
        if not any(_identity(pid) == self.members[pid][1] for pid in self.live()):
            return
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():
                continue
            pid = int(proc.name)
            if pid in self.members:
                continue
            identity = _identity(pid)
            if identity is not None and identity[:2] == (os.getuid(), self.sid):
                self._remember(pid, identity)

    def _signal(self, number, signalled):
        for pid, fd in self.live().items():
            try:
                signal.pidfd_send_signal(fd, number)
                signalled.add(pid)
            except ProcessLookupError:
                pass

    def stop(self, grace_s=5.0, kill_wait_s=5.0):
        self.refresh()
        terminated, killed = set(), set()
        self._signal(signal.SIGTERM, terminated)
        deadline = time.monotonic() + grace_s
        while self.live() and time.monotonic() < deadline:
            self.refresh()
            # Newly discovered descendants also receive the graceful signal.
            for pid, fd in self.live().items():
                if pid not in terminated:
                    try:
                        signal.pidfd_send_signal(fd, signal.SIGTERM)
                        terminated.add(pid)
                    except ProcessLookupError:
                        pass
            time.sleep(.025)
        self.refresh()
        self._signal(signal.SIGKILL, killed)
        deadline = time.monotonic() + kill_wait_s
        while self.live() and time.monotonic() < deadline:
            self.refresh()
            self._signal(signal.SIGKILL, killed)
            time.sleep(.025)
        self.process.poll()  # Reap our direct child even if it exited earlier.
        remaining = sorted(self.live())
        return dict(session_id=self.sid, tracked_pids=sorted(self.members),
                    term_pids=sorted(terminated), kill_pids=sorted(killed),
                    remaining_live_pids=remaining, all_tracked_exited=not remaining,
                    scope='Owned session descendants; children that create a new session are excluded')

    def close(self):
        for fd, _ in self.members.values():
            os.close(fd)
        self.members.clear()
