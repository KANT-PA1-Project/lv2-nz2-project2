"""Single-slot shared-memory transport for opt-in stationary diagnostics.

A producer publishes a descriptor only after copying a complete frame. It cannot
reuse the slot until an exact acknowledgement arrives. Reader snapshots own their
arrays; no view survives slot reuse. Socket/pipe descriptor delivery is the handoff.
"""
import mmap
import os
import secrets
import tempfile
from pathlib import Path
import numpy as np


class SharedFrameSlot:
    def __init__(self, capacity=32 * 1024 * 1024, *, directory=None):
        if type(capacity) is not int or not 0 < capacity <= 64 * 1024 * 1024:
            raise ValueError('invalid_capacity')
        self.directory = None
        if directory is None:
            self.directory = tempfile.TemporaryDirectory(prefix='r12-frame-', dir='/dev/shm')
            directory = self.directory.name
        parent = Path(directory)
        if parent.is_symlink() or parent.parent != Path('/dev/shm') or not parent.name.startswith('r12-frame-') or parent.stat().st_uid != os.getuid():
            raise ValueError('invalid_supervised_directory')
        self.path = parent / 'slot'
        self.fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
        os.ftruncate(self.fd, capacity)
        self.buffer = mmap.mmap(self.fd, capacity)
        self.capacity = capacity
        self.pending = None
        self.sequence = 0
        self.session = secrets.token_hex(16)
        self.closed = False

    def publish(self, arrays):
        if self.closed or self.pending is not None:
            raise ValueError('closed_or_unacknowledged_slot')
        entries = []; offset = 0
        for key, value in arrays.items():
            a = np.asarray(value)
            if not isinstance(key, str) or a.dtype.kind not in 'biufU' or a.dtype.hasobject:
                raise ValueError('unsupported_member')
            blob = a.tobytes(order='C')
            entries.append((dict(key=key, dtype=a.dtype.str, shape=list(a.shape),
                                 offset=offset, nbytes=len(blob)), blob))
            offset += len(blob)
        if offset > self.capacity or not entries:
            raise ValueError('frame_exceeds_capacity_or_empty')
        for entry, blob in entries:
            self.buffer[entry['offset']:entry['offset'] + entry['nbytes']] = blob
        self.sequence += 1
        token = secrets.token_hex(16)
        self.pending = dict(session=self.session, sequence=self.sequence, token=token)
        return dict(schema='r12_shared_frame_prototype_v1', path=str(self.path),
                    size=offset, entries=[e for e, _ in entries], **self.pending)

    def acknowledge(self, ack):
        if self.closed or self.pending is None or ack != self.pending:
            raise ValueError('invalid_acknowledgement')
        self.pending = None

    def close(self):
        if not self.closed:
            self.closed = True
            self.buffer.close(); os.close(self.fd)
            self.path.unlink()
            if self.directory is not None:self.directory.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def snapshot(descriptor):
    if descriptor['schema'] != 'r12_shared_frame_prototype_v1':
        raise ValueError('invalid_schema')
    path = Path(descriptor['path'])
    if path.parent.parent != Path('/dev/shm') or not path.parent.name.startswith('r12-frame-') or path.name != 'slot':
        raise ValueError('invalid_slot_path')
    size = descriptor['size']
    if type(size) is not int or not 0 < size <= 64 * 1024 * 1024:
        raise ValueError('invalid_size')
    arrays = {}; expected_offset = 0
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        st = os.fstat(fd)
        if st.st_uid != os.getuid() or st.st_size < size:
            raise ValueError('invalid_slot_owner_or_size')
        with mmap.mmap(fd, size, access=mmap.ACCESS_READ) as buf:
            for e in descriptor['entries']:
                dtype = np.dtype(e['dtype']); shape = e['shape']
                if dtype.hasobject or dtype.kind not in 'biufU' or any(type(v) is not int or v < 0 for v in shape):
                    raise ValueError('invalid_dtype_or_shape')
                count = 1
                for value in shape:
                    count *= value
                nbytes = count * dtype.itemsize
                if e['key'] in arrays or e['offset'] != expected_offset or nbytes != e['nbytes'] or expected_offset + nbytes > size:
                    raise ValueError('invalid_member_bounds')
                # bytes + copy owns storage, independent of producer overwrite after ACK.
                a = np.frombuffer(buf[expected_offset:expected_offset+nbytes], dtype=dtype).reshape(shape).copy()
                a.flags.writeable = False
                arrays[e['key']] = a
                expected_offset += nbytes
        if expected_offset != size:
            raise ValueError('incomplete_frame')
    finally:
        os.close(fd)
    return arrays, {key: descriptor[key] for key in ('session', 'sequence', 'token')}
