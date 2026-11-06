"""Synchronous wire views of sealed owning arrays, without an extra byte copy."""
import numpy as np


def immutable_blob(array):
    if not isinstance(array, np.ndarray) or not array.flags.owndata or not array.flags.c_contiguous:
        raise ValueError("wire_requires_owned_contiguous_array")
    if array.flags.writeable:
        raise ValueError("wire_requires_sealed_readonly_array")
    # The view retains its ndarray owner until synchronous sendall has returned.
    return memoryview(array).cast("B").toreadonly()
