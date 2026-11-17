"""Byte identity and ownership for inactive owned RGB copy candidate."""
import numpy as np
import pytest
from arm_practice.native_rgbd.owned_rgb import owned_rgb

@pytest.mark.parametrize('channels',[3,4])
@pytest.mark.parametrize('dtype',[np.uint8,np.float32])
@pytest.mark.parametrize('layout',['contiguous','stride','reverse','transpose'])
@pytest.mark.parametrize('readonly',[False,True])
def test_exact_bytes_and_independent_storage(channels,dtype,layout,readonly):
 rng=np.random.default_rng(1815)
 backing=rng.integers(0,256,(48,64,channels),dtype=np.uint8).astype(dtype)
 a={'contiguous':backing,'stride':backing[:,::2], 'reverse':backing[::-1], 'transpose':backing.transpose(1,0,2)}[layout]
 if readonly:a.flags.writeable=False
 before=a[:,:,:3].tobytes();original_flags=a.flags.writeable
 result=owned_rgb(a)
 assert result.tobytes()==before and result.dtype==a.dtype
 assert result.flags.owndata and not np.shares_memory(a,result)
 assert a.flags.writeable==original_flags
 if a is not backing or not readonly:
  backing[:]=0
  assert result.tobytes()==before
 result[:]=7
 if readonly:assert not a.flags.writeable

def test_alpha_does_not_change_rgb():
 rng=np.random.default_rng(8);a=rng.integers(0,256,(23,17,4),dtype=np.uint8)
 first=owned_rgb(a);a[:,:,3]^=255
 assert np.array_equal(first,owned_rgb(a))

def test_float_nan_payload_bits_preserved():
 bits=np.array([0x7fc00001,0x7fc00002,0x80000000,0x3f800000],dtype=np.uint32)
 a=np.tile(bits,(5,7,1)).view(np.float32)
 assert owned_rgb(a).tobytes()==a[:,:,:3].copy().tobytes()
