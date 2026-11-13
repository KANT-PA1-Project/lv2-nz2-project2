#!/usr/bin/env python3
"""Extract CAD data via libarchive and assemble a simulation-only URDF."""
import argparse
import ctypes as C
import ctypes.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from common import assemble_urdf, load_config, sha256
from appearance import asset_paths


def extract_data(archive_path, destination):
    """No archive script execution, symlinks, path traversal or overwrites."""
    lib = C.CDLL(ctypes.util.find_library("archive") or "libarchive.so.13")
    signatures = {
        "archive_read_new": ([], C.c_void_p),
        "archive_read_support_format_all": ([C.c_void_p], C.c_int),
        "archive_read_support_filter_all": ([C.c_void_p], C.c_int),
        "archive_read_open_filename": ([C.c_void_p, C.c_char_p, C.c_size_t], C.c_int),
        "archive_read_next_header": ([C.c_void_p, C.POINTER(C.c_void_p)], C.c_int),
        "archive_entry_pathname": ([C.c_void_p], C.c_char_p),
        "archive_entry_filetype": ([C.c_void_p], C.c_uint),
        "archive_entry_size": ([C.c_void_p], C.c_longlong),
        "archive_read_data": ([C.c_void_p, C.c_void_p, C.c_size_t], C.c_ssize_t),
        "archive_error_string": ([C.c_void_p], C.c_char_p),
        "archive_read_free": ([C.c_void_p], C.c_int),
    }
    for name, (args, result) in signatures.items():
        fn = getattr(lib, name)
        fn.argtypes, fn.restype = args, result
    reader = lib.archive_read_new()
    if not reader:
        raise RuntimeError("libarchive allocation failed")
    total = 0
    try:
        lib.archive_read_support_format_all(reader)
        lib.archive_read_support_filter_all(reader)
        if lib.archive_read_open_filename(reader, str(archive_path).encode(), 65536) != 0:
            raise RuntimeError(lib.archive_error_string(reader))
        entry, buffer = C.c_void_p(), C.create_string_buffer(65536)
        base = Path(destination).resolve()
        while True:
            status = lib.archive_read_next_header(reader, C.byref(entry))
            if status == 1:  # ARCHIVE_EOF
                break
            if status != 0:
                raise RuntimeError(lib.archive_error_string(reader))
            relative = Path(lib.archive_entry_pathname(entry).decode("utf-8"))
            target = (base / relative).resolve()
            if relative.is_absolute() or base not in target.parents:
                raise ValueError(f"Unsafe archive path: {relative}")
            kind = lib.archive_entry_filetype(entry)
            if kind == 0o040000:
                target.mkdir(parents=True, exist_ok=True)
                continue
            if kind != 0o100000:
                raise ValueError(f"Non-regular archive member: {relative}")
            size = lib.archive_entry_size(entry)
            total += size
            if size < 0 or size > 256 * 1024**2 or total > 512 * 1024**2:
                raise ValueError("Archive exceeds data extraction budget")
            # Only runtime assets and provenance. Never unpack/execute scripts.
            if not (relative.parts[0] == "R20D" and len(relative.parts) >= 2
                    and relative.parts[1] in {"urdf", "meshes", "checks", "SHA256.json", "README_KO.md"}):
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            written = 0
            with target.open("xb") as stream:
                while True:
                    n = lib.archive_read_data(reader, buffer, len(buffer))
                    if n < 0:
                        raise RuntimeError(lib.archive_error_string(reader))
                    if not n:
                        break
                    written += n
                    if written > size:
                        raise ValueError("Archive member size mismatch")
                    stream.write(buffer.raw[:n])
            if written != size:
                raise ValueError("Truncated archive member")
    finally:
        lib.archive_read_free(reader)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    ap.add_argument("--out", required=True, type=Path, help="New output directory, outside source assets")
    args = ap.parse_args()
    cfg, _ = load_config(args.config)  # fail before extracting if mount/poses unknown
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    extract_data(cfg["r20d_archive"], out / "cad")
    manifest = assemble_urdf(cfg, out / "cad/R20D", out / "r20d_perception.urdf")
    mesh_paths = {mesh.get('filename') for mesh in ET.parse(out / 'r20d_perception.urdf').iter('mesh')}
    manifest.update(archive_sha256=sha256(cfg["r20d_archive"]), config=cfg,
                    appearance_sha256={str(path): sha256(path) for path in asset_paths(cfg)},
                    mesh_sha256={path: sha256(path) for path in sorted(mesh_paths)},
                    status="assembled_not_isaac_validated")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print(out / "r20d_perception.urdf")


if __name__ == "__main__":
    main()
