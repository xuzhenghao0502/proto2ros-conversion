# Copyright (c) 2023 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# ruff: noqa
# fmt: off

# NOTE(mhidalgo): handle https://bugs.launchpad.net/ubuntu/+source/networkx/+bug/2002660
import sys

import numpy
numpy.int = numpy.int_

try:
    import networkx
except ValueError as exc:
    # Common when pip NumPy and apt SciPy (pulled in via networkx -> generators) are ABI-mismatched.
    msg = str(exc)
    if 'numpy.dtype size changed' in msg or 'binary incompatibility' in msg:
        print(
            'proto2ros: NumPy 与 SciPy 二进制不兼容（常见于混用 pip 的 numpy 与 apt 的 scipy/networkx）。\n'
            '任选其一修复后重新构建：\n'
            '  python3 -m pip install -U "numpy>=1.26" scipy networkx\n'
            '或重装系统包并避免再用 pip 覆盖 numpy：\n'
            '  sudo apt install --reinstall python3-numpy python3-scipy python3-networkx\n',
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    raise
