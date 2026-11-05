import importlib.util
from pathlib import Path
import subprocess
import sys
import time

MODULE = Path(__file__).resolve().parents[1] / 'scripts/r12_owned_session.py'
spec = importlib.util.spec_from_file_location('r12_owned_session', MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def wait_for(predicate, seconds=5):
    end = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < end, 'Process fixture did not become ready'
        time.sleep(.025)


def test_cleanup_surviving_descendant_after_leader_exit(tmp_path):
    ready, release = tmp_path / 'ready', tmp_path / 'release'
    child = ('import signal,time; from pathlib import Path; '
             'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
             f'Path({str(ready)!r}).write_text("ready"); time.sleep(60)')
    parent = ('import subprocess,sys,time; from pathlib import Path; '
              f'subprocess.Popen([sys.executable,"-c",{child!r}]); '
              f'p=Path({str(release)!r})\n'
              'while not p.exists(): time.sleep(.025)\n')
    process = subprocess.Popen([sys.executable, '-c', parent], start_new_session=True)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'],
                                 start_new_session=True)
    owned = module.OwnedSession(process)
    try:
        wait_for(ready.exists)
        owned.refresh()
        descendants = set(owned.live()) - {process.pid}
        assert len(descendants) == 1
        release.touch()
        process.wait(timeout=5)
        assert descendants <= set(owned.live())
        report = owned.stop(grace_s=.1)
        assert report['all_tracked_exited']
        assert set(report['kill_pids']) == descendants
        assert unrelated.poll() is None
    finally:
        owned.stop(grace_s=.1)
        owned.close()
        unrelated.terminate()
        unrelated.wait(timeout=5)


def test_empty_session_not_rediscovered(monkeypatch):
    process = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'],
                               start_new_session=True)
    owned = module.OwnedSession(process)
    try:
        process.terminate()
        process.wait(timeout=5)
        def unexpected_scan(*args):
            raise AssertionError('Must not rediscover an empty session by numeric ID')
        monkeypatch.setattr(module.Path, 'iterdir', unexpected_scan)
        owned.refresh()
        assert owned.stop(grace_s=.1)['all_tracked_exited']
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        owned.close()
