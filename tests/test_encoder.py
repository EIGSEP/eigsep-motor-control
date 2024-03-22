import struct
import numpy as np
import pytest

import eigsep_motor_control as emc


def _generate_data(seed):
    """Generate analog data from a Pico"""
    rng = np.random.default_rng(seed=seed)
    size = emc.serial_params.INT_LEN
    val = rng.integers(0, 2**16, size=size, dtype=np.uint16, endpoint=False)
    return np.sum(val)


@pytest.mark.parametrize("seed", [0, 42, 2024, 7385])
def test_read_analog(seed):
    # generate fake data
    v1 = _generate_data(seed)
    v2 = _generate_data(seed)
    read = bytearray()
    read.extend(struct.pack("<ii", v1, v2))
    # read data
    val = np.array(struct.unpack("<ii", read))
    assert np.all(val == [v1, v2])
