"""CPU checks for final metadata; never import or launch Isaac."""
import ast
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent


def metadata_finalizer():
    tree = ast.parse((HERE / "run.py").read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    guarded_run = next(n for n in main.body if isinstance(n, ast.Try) and n.finalbody)
    calls = [n for n in guarded_run.finalbody if isinstance(n, ast.Expr)
             and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Attribute)
             and isinstance(n.value.func.value, ast.Name) and n.value.func.value.id == "result"
             and n.value.func.attr == "update"]
    assert len(calls) == 1
    return compile(ast.Module(body=calls, type_ignores=[]), str(HERE / "run.py"), "exec")


def test_tower_first_includes_two_second_done_hold():
    env = dict(result={"frame_count": 1175, "wall_s": 592.}, logged_frames=1235,
               time=SimpleNamespace(monotonic=lambda: 610.), wall_start=10.)
    exec(metadata_finalizer(), env)
    assert env["result"] == {"frame_count": 1235, "wall_s": 600.}


def test_barcode_first_final_count_stays_correct():
    env = dict(result={"frame_count": 1701}, logged_frames=1701,
               time=SimpleNamespace(monotonic=lambda: 1260.), wall_start=10.)
    exec(metadata_finalizer(), env)
    assert env["result"]["frame_count"] == 1701


def test_failure_before_first_frame_reports_zero():
    env = dict(result={"status": "failed"}, logged_frames=0,
               time=SimpleNamespace(monotonic=lambda: 12.), wall_start=10.)
    exec(metadata_finalizer(), env)
    assert env["result"] == {"status": "failed", "frame_count": 0, "wall_s": 2.}
