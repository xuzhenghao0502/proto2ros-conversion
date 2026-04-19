# Copyright 2026 GPAL — parse EkaRT ``eka::rt::msg::PointCloud`` zerocopy blobs (GPAL layout).
"""Shared parser for lidar zerocopy rows (ASCII field tag + metadata + packed points)."""

from __future__ import annotations

import struct
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

EKA_SER_POINT_CLOUD = 'eka::rt::msg::PointCloud'

# Zerocopy stores a 32-bit ``intensity`` word: reflectivity is in the **low 8 bits**; high bits are
# device flags (e.g. 0xF0……), so uint32-as-is (4026531842) looks wrong. ROS ``PointCloud2`` exports
# ``intensity`` as **float32(float(word & mask))** (typ. 0..255). For 0..1 normalization divide by
# 255 in your node or RViz. Widen ``INTENSITY_VALUE_MASK`` if firmware uses more than 8 bits.
INTENSITY_VALUE_MASK = 0xFF

PC_LAYOUT_MARK = b'x,y,z,intensity,timestamp_offset,ring\x00'


def _struct_char_for_tail_field(name: str) -> str:
    if name == 'intensity':
        return 'I'  # uint32 LE wire format in zerocopy
    return 'i'


def _ros_tail_struct_chars(tail_packed_names: Tuple[str, ...]) -> str:
    """Struct chars for repacked ROS point (intensity → float32, rest int32)."""
    return ''.join('f' if n == 'intensity' else 'i' for n in tail_packed_names)


def _resolve_tail_layout(
    tail_names: List[str],
    tail_len: int,
) -> Optional[Tuple[str, Tuple[str, ...]]]:
    """Match a prefix of ``tail_names`` whose encoded size equals ``tail_len``.

    Returns ``(struct_suffix, packed_names)`` for use after ``'<fff'``, e.g. ``('Ii', ...)``.
    """
    if tail_len < 0 or tail_len % 4 != 0:
        return None
    chars: List[str] = []
    packed: List[str] = []
    total = 0
    for name in tail_names:
        c = _struct_char_for_tail_field(name)
        sz = struct.calcsize('<' + c)
        if total + sz > tail_len:
            break
        chars.append(c)
        packed.append(name)
        total += sz
        if total == tail_len:
            return ''.join(chars), tuple(packed)
    return None


def _find_layout_ascii(blob: bytes) -> Optional[Tuple[int, str]]:
    """Locate ``x,y,z,...`` field list ending at the first ``\\0``."""
    i = blob.find(b'x,y,z,')
    if i < 0:
        return None
    j = blob.find(b'\x00', i)
    if j < 0:
        return None
    s = blob[i:j].decode('ascii', errors='replace')
    return i, s


def _tail_names_from_layout(layout_ascii: str) -> List[str]:
    parts = [p.strip() for p in layout_ascii.split(',') if p.strip()]
    if len(parts) < 4 or parts[0:3] != ['x', 'y', 'z']:
        return []
    return parts[3:]


def sniff_pointcloud_blob(blob: bytes) -> str:
    if _find_layout_ascii(blob) is not None:
        return 'gpal_zerocopy_marker'
    return 'no_layout_marker'


def intensity_u32_to_ros_float(raw_u32: int) -> float:
    """Map zerocopy intensity word to ``sensor_msgs`` float (low bits, typ. 0..255)."""
    return float(int(raw_u32) & INTENSITY_VALUE_MASK)


@dataclass(frozen=True)
class LiDARPoint:
    """Wire-format ``intensity`` word plus ROS-style normalized intensity."""

    index: int
    x: float
    y: float
    z: float
    intensity_raw: int  # full uint32 word from zerocopy
    intensity: float  # float(intensity_raw & mask), typ. 0..255
    timestamp_offset: int
    ring: Optional[int] = None
    echo_number: Optional[int] = None

    def describe_lines(self) -> List[str]:
        lo = self.timestamp_offset & 0xFFFF
        hi = (self.timestamp_offset >> 16) & 0xFFFF
        low8 = self.intensity_raw & INTENSITY_VALUE_MASK
        lines = [
            f'  index: {self.index}',
            f'  x (float32): {self.x!r}',
            f'  y (float32): {self.y!r}',
            f'  z (float32): {self.z!r}',
            f'  intensity_raw (uint32 LE): {self.intensity_raw} (0x{self.intensity_raw & 0xFFFFFFFF:08x})',
            f'  intensity low bits (& 0x{INTENSITY_VALUE_MASK:X}): {low8}',
            f'  intensity (ROS float, low bits): {self.intensity!r}',
            f'  timestamp_offset (int32 LE): {self.timestamp_offset} (0x{self.timestamp_offset & 0xFFFFFFFF:08x})',
            f'  ring: {self.ring!r}',
            f'  echo_number: {self.echo_number!r}',
            f'  speculative unpack of timestamp_offset: uint16_low={lo} uint16_high={hi} (verify on device)',
        ]
        return lines


@dataclass(frozen=True)
class ZerocopyPointcloudParsed:
    """Result of parsing one DB ``data`` blob for the GPAL zerocopy layout."""

    marker_offset: int
    layout_ascii: str
    tail_field_names: Tuple[str, ...]
    point_step: int
    tail_struct_suffix: str
    tail_packed_names: Tuple[str, ...]
    meta_offset: int
    meta_u32: Tuple[int, ...]
    point_count_header: int
    point_count_effective: int
    data_start: int
    tail_pad_bytes: int
    blob_total_len: int
    raw_point_block: bytes

    def lidar_points(self) -> List[LiDARPoint]:
        return _unpack_points(
            self.raw_point_block,
            self.point_step,
            self.tail_struct_suffix,
            self.tail_packed_names,
        )

    def lidar_point_at(self, index: int) -> Optional[LiDARPoint]:
        pts = self.lidar_points()
        if 0 <= index < len(pts):
            return pts[index]
        return None


def _unpack_points(
    raw: bytes,
    step: int,
    tail_struct_suffix: str,
    tail_packed_names: Tuple[str, ...],
) -> List[LiDARPoint]:
    n = len(raw) // step
    row_fmt = '<fff' + tail_struct_suffix
    out: List[LiDARPoint] = []
    for i in range(n):
        o = i * step
        vals = struct.unpack_from(row_fmt, raw, o)
        x, y, z = float(vals[0]), float(vals[1]), float(vals[2])
        rest = dict(zip(tail_packed_names, vals[3:]))
        raw_i = int(rest.get('intensity', 0))
        out.append(
            LiDARPoint(
                index=i,
                x=x,
                y=y,
                z=z,
                intensity_raw=raw_i,
                intensity=intensity_u32_to_ros_float(raw_i),
                timestamp_offset=int(rest.get('timestamp_offset', 0)),
                ring=rest.get('ring'),
                echo_number=rest.get('echo_number'),
            )
        )
    return out


def _repack_raw_block_to_ros_pointcloud2_data(
    raw: bytes,
    step: int,
    tail_struct_suffix: str,
    tail_packed_names: Tuple[str, ...],
) -> Tuple[bytes, int]:
    """Rebuild binary with float32 ``intensity``; returns (new_data, new_point_step)."""
    row_in = '<fff' + tail_struct_suffix
    row_out = '<fff' + _ros_tail_struct_chars(tail_packed_names)
    new_step = struct.calcsize(row_out)
    n = len(raw) // step
    buf = bytearray()
    for i in range(n):
        o = i * step
        vals = list(struct.unpack_from(row_in, raw, o))
        for j, name in enumerate(tail_packed_names):
            if name == 'intensity':
                idx = 3 + j
                vals[idx] = intensity_u32_to_ros_float(int(vals[idx]))
        buf.extend(struct.pack(row_out, *vals))
    return bytes(buf), new_step


def _pick_point_step(
    payload_len: int,
    count_hdr: int,
    tail_names: List[str],
) -> Optional[Tuple[int, int, int]]:
    """Return ``(point_step, count_effective, tail_pad)`` or ``None``."""
    candidates = [32, 28, 24, 20]
    if 'echo_number' in tail_names:
        candidates = [28, 24, 32, 20]
    for step in candidates:
        tail_len = step - 12
        if _resolve_tail_layout(tail_names, tail_len) is None:
            continue
        for tail_pad in (4, 0, 8):
            if payload_len == count_hdr * step + tail_pad:
                return step, count_hdr, tail_pad
    for step in candidates:
        tail_len = step - 12
        if _resolve_tail_layout(tail_names, tail_len) is None:
            continue
        for tail_pad in (4, 0, 8):
            rem = payload_len - tail_pad
            if rem > 0 and rem % step == 0:
                return step, rem // step, tail_pad
    return None


def parse_zerocopy_pointcloud_blob(blob: bytes) -> Optional[ZerocopyPointcloudParsed]:
    """Parse blob; on failure print to stderr and return ``None``."""
    found = _find_layout_ascii(blob)
    if found is None:
        print(
            f'[warn] PointCloud: missing GPAL layout marker (blob_hint={sniff_pointcloud_blob(blob)!r}); skip',
            file=sys.stderr,
        )
        return None
    idx, layout_ascii = found
    tail_names = _tail_names_from_layout(layout_ascii)
    if not tail_names:
        print(f'[warn] PointCloud: bad layout ASCII {layout_ascii!r}', file=sys.stderr)
        return None

    p = idx + len(layout_ascii) + 1
    while p < len(blob) and blob[p] == 0:
        p += 1
    meta_base = p
    if meta_base + 48 > len(blob):
        print('[warn] PointCloud: metadata truncated', file=sys.stderr)
        return None
    meta_u32 = struct.unpack_from('<' + 'I' * 12, blob, meta_base)
    count_hdr = int(meta_u32[6])
    data_start = meta_base + 48
    payload_len = len(blob) - data_start

    picked = _pick_point_step(payload_len, count_hdr, tail_names)
    if picked is None:
        print('[warn] PointCloud: size / count mismatch', file=sys.stderr)
        return None
    step, count_eff, tail_pad = picked
    tail_len = step - 12
    resolved = _resolve_tail_layout(tail_names, tail_len)
    if resolved is None:
        print(
            f'[warn] PointCloud: cannot map tail fields {tail_names!r} to {tail_len} bytes',
            file=sys.stderr,
        )
        return None
    tail_struct_suffix, tail_packed_names = resolved

    raw = blob[data_start : data_start + count_eff * step]
    return ZerocopyPointcloudParsed(
        marker_offset=idx,
        layout_ascii=layout_ascii,
        tail_field_names=tuple(tail_names),
        point_step=step,
        tail_struct_suffix=tail_struct_suffix,
        tail_packed_names=tail_packed_names,
        meta_offset=meta_base,
        meta_u32=tuple(int(x) for x in meta_u32),
        point_count_header=count_hdr,
        point_count_effective=count_eff,
        data_start=data_start,
        tail_pad_bytes=tail_pad,
        blob_total_len=len(blob),
        raw_point_block=raw,
    )


def pointcloud2_from_parsed(parsed: ZerocopyPointcloudParsed, header: Header) -> PointCloud2:
    """Build ``sensor_msgs/PointCloud2`` with float32 ``intensity`` from low bits (typ. 0..255)."""
    n = parsed.point_count_effective
    new_data, new_step = _repack_raw_block_to_ros_pointcloud2_data(
        parsed.raw_point_block,
        parsed.point_step,
        parsed.tail_struct_suffix,
        parsed.tail_packed_names,
    )
    pc = PointCloud2()
    pc.header = header
    pc.height = 1
    pc.width = int(n)
    pc.is_dense = False
    pc.is_bigendian = False
    fields: List[PointField] = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    off = 12
    ros_field = {
        'intensity': 'intensity',
        'timestamp_offset': 'timestamp_offset',
        'ring': 'ring',
        'echo_number': 'echo_number',
    }
    for nm in parsed.tail_packed_names:
        ros_nm = ros_field.get(nm, nm)
        dt = PointField.FLOAT32 if nm == 'intensity' else PointField.INT32
        fields.append(PointField(name=ros_nm, offset=off, datatype=dt, count=1))
        off += 4
    pc.fields = fields
    pc.point_step = new_step
    pc.row_step = new_step * int(n)
    pc.data = new_data
    return pc
