"""Finalize duration metadata for this viewer's Chromium canvas recordings.

This deliberately is not a general WebM remuxer. It accepts one VP8/VP9 video
track in an unknown-size Segment, Info + Tracks + video-block Clusters, with
Chrome/Chromium muxer names. Indexed files, position metadata, checksummed Info,
other tracks, and unfamiliar layouts are rejected before any bytes are changed.
Encoded frames, timestamps, Tracks and Clusters are copied byte for byte.

Matroska Duration uses Segment ticks; TimestampScale is nanoseconds per tick:
https://www.matroska.org/technical/elements.html
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import struct


EBML, SEGMENT, INFO, TRACKS, CLUSTER = 0x1A45DFA3, 0x18538067, 0x1549A966, 0x1654AE6B, 0x1F43B675
TIMESTAMP_SCALE, DURATION = 0x2AD7B1, 0x4489
_TOP_LEVEL = {INFO, TRACKS, CLUSTER, 0x114D9B74, 0x1C53BB6B, 0x1941A469, 0x1043A770, 0x1254C367}


@dataclass(frozen=True)
class Element:
    id: int
    start: int
    data_start: int
    end: int
    unknown_size: bool = False


def _vint(data: bytes, offset: int, limit: int, *, element_id: bool = False) -> tuple[int, int, bool]:
    if offset >= limit or data[offset] == 0:
        raise ValueError("truncated or invalid EBML variable integer")
    marker, width = 0x80, 1
    while not data[offset] & marker:
        marker >>= 1
        width += 1
    if width > (4 if element_id else 8) or offset + width > limit:
        raise ValueError("invalid EBML integer width")
    raw = int.from_bytes(data[offset:offset + width], "big")
    if element_id:
        return raw, width, False
    value = raw & ((1 << (7 * width)) - 1)
    return value, width, value == (1 << (7 * width)) - 1


def _element(data: bytes, offset: int, limit: int) -> Element:
    identifier, id_width, _ = _vint(data, offset, limit, element_id=True)
    size, size_width, unknown = _vint(data, offset + id_width, limit)
    begin = offset + id_width + size_width
    end = limit if unknown else begin + size
    if end > limit:
        raise ValueError("EBML element extends past its parent or end of file")
    return Element(identifier, offset, begin, end, unknown)


def _children(data: bytes, parent: Element):
    offset = parent.data_start
    while offset < parent.end:
        child = _element(data, offset, parent.end)
        if child.unknown_size:
            raise ValueError("unexpected unknown-size metadata element")
        yield child
        offset = child.end


def _size(value: int) -> bytes:
    for width in range(1, 9):
        if value < (1 << (7 * width)) - 1:
            return ((1 << (7 * width)) | value).to_bytes(width, "big")
    raise ValueError("EBML element is too large")


def _uint(data: bytes, element: Element) -> int:
    if not 1 <= element.end - element.data_start <= 8:
        raise ValueError("invalid unsigned metadata integer")
    return int.from_bytes(data[element.data_start:element.end], "big")


def _video_track(data: bytes, tracks: Element) -> int:
    entries = list(_children(data, tracks))
    if len(entries) != 1 or entries[0].id != 0xAE:
        raise ValueError("expected exactly one MediaRecorder video track")
    fields = {item.id: item for item in _children(data, entries[0])}
    if any(key not in fields for key in (0xD7, 0x83, 0x86)):
        raise ValueError("video track metadata is incomplete")
    codec = fields[0x86]
    if _uint(data, fields[0x83]) != 1 or data[codec.data_start:codec.end] not in (b"V_VP8", b"V_VP9"):
        raise ValueError("only VP8/VP9 canvas video recordings are supported")
    return _uint(data, fields[0xD7])


def _block_timestamp(data: bytes, block: Element, track: int, timestamp: int) -> int:
    number, width, unknown = _vint(data, block.data_start, block.end)
    if number != track or unknown or block.end - block.data_start < width + 4:
        raise ValueError("invalid or unexpected video block track")
    relative = int.from_bytes(data[block.data_start + width:block.data_start + width + 2],
                              "big", signed=True)
    if data[block.data_start + width + 2] & 0x06:
        raise ValueError("laced frames are outside this recorder format")
    return timestamp + relative


def _block_group(data: bytes, group: Element, track: int, timestamp: int) -> int:
    fields = list(_children(data, group))
    # References, durations, discard padding and VP9 BlockAdditions describe
    # frames, never positions in the containing Segment. Preserve them all.
    if any(item.id not in {0xA1, 0x9B, 0xFB, 0x75A2, 0x75A1} for item in fields):
        raise ValueError("unsupported BlockGroup metadata")
    blocks = [item for item in fields if item.id == 0xA1]
    if len(blocks) != 1:
        raise ValueError("expected one Block per BlockGroup")
    for additions in (item for item in fields if item.id == 0x75A1):
        for more in _children(data, additions):
            if more.id != 0xA6 or any(item.id not in (0xEE, 0xA5) for item in _children(data, more)):
                raise ValueError("unsupported BlockAdditions metadata")
    return _block_timestamp(data, blocks[0], track, timestamp)


def _cluster(data: bytes, cluster: Element, track: int) -> tuple[int, list[int]]:
    offset, timestamp, stamps = cluster.data_start, None, []
    while offset < cluster.end:
        child = _element(data, offset, cluster.end)
        if child.id in _TOP_LEVEL:
            if cluster.unknown_size:
                break
            raise ValueError("top-level element appears inside a finite Cluster")
        if child.unknown_size:
            raise ValueError("unknown-size Cluster child is unsupported")
        if child.id == 0xE7:
            if timestamp is not None or stamps:
                raise ValueError("Cluster Timestamp must appear once before its frames")
            timestamp = _uint(data, child)
        elif child.id in (0xA3, 0xA0):
            if timestamp is None:
                raise ValueError("video block has no Cluster Timestamp")
            stamps.append(_block_timestamp(data, child, track, timestamp) if child.id == 0xA3
                          else _block_group(data, child, track, timestamp))
        elif child.id != 0xEC:  # Void has no offsets or timing effect.
            raise ValueError("unsupported Cluster metadata; refusing to shift file offsets")
        offset = child.end
    if timestamp is None or not stamps:
        raise ValueError("expected a Timestamp and video frames in every Cluster")
    return offset, stamps


def finalize_webm(data: bytes, duration_hint_seconds: float | None = None) -> tuple[bytes, float]:
    """Return finalized bytes and seconds, preserving the encoded video.

    Infer the final frame's end from encoded timestamps plus the median frame
    interval. A positive replay duration hint supplies a minimum duration.
    """
    if duration_hint_seconds is not None and (
            not math.isfinite(duration_hint_seconds) or duration_hint_seconds <= 0):
        raise ValueError("duration hint must be finite and positive")
    if not isinstance(data, bytes):
        raise ValueError("WebM data must be bytes")
    header = _element(data, 0, len(data))
    if header.id != EBML or header.unknown_size:
        raise ValueError("expected a finite EBML header")
    doc_types = [item for item in _children(data, header) if item.id == 0x4282]
    if len(doc_types) != 1 or data[doc_types[0].data_start:doc_types[0].end] != b"webm":
        raise ValueError("expected WebM DocType")
    segment = _element(data, header.end, len(data))
    if segment.id != SEGMENT or not segment.unknown_size:
        raise ValueError("only unindexed unknown-size MediaRecorder Segments are supported")
    info = _element(data, segment.data_start, segment.end)
    if info.id != INFO or info.unknown_size:
        raise ValueError("expected Info first; indexed or unfamiliar WebM files are unsupported")
    fields = list(_children(data, info))
    allowed = {TIMESTAMP_SCALE, DURATION, 0x4D80, 0x5741}
    if any(item.id not in allowed for item in fields) or len({item.id for item in fields}) != len(fields):
        raise ValueError("unsupported or duplicate Info metadata")
    apps = {item.id: data[item.data_start:item.end] for item in fields if item.id in (0x4D80, 0x5741)}
    if len(apps) != 2 or any(name not in (b"Chrome", b"Chromium") for name in apps.values()):
        raise ValueError("only Chrome/Chromium MediaRecorder files are supported")
    scale = next((_uint(data, item) for item in fields if item.id == TIMESTAMP_SCALE), 1_000_000)
    if scale <= 0:
        raise ValueError("TimestampScale must be positive")
    tracks = _element(data, info.end, segment.end)
    if tracks.id != TRACKS or tracks.unknown_size:
        raise ValueError("expected Tracks directly after Info")
    track = _video_track(data, tracks)
    offset, stamps = tracks.end, []
    while offset < segment.end:
        cluster = _element(data, offset, segment.end)
        if cluster.id != CLUSTER:
            raise ValueError("indexed/offset metadata is unsupported; only Clusters may follow Tracks")
        offset, cluster_stamps = _cluster(data, cluster, track)
        stamps.extend(cluster_stamps)
    if not stamps or min(stamps) < 0:
        raise ValueError("recording has no valid video timestamps")
    ordered = sorted(set(stamps))
    intervals = [b - a for a, b in zip(ordered, ordered[1:]) if b > a]
    if intervals:
        frame_ticks = statistics.median(intervals)
    elif duration_hint_seconds is not None:
        frame_ticks = duration_hint_seconds * 1_000_000_000 / scale
    else:
        raise ValueError("a single-frame recording requires a duration hint")
    duration = max((ordered[-1] + frame_ticks) * scale / 1_000_000_000,
                   duration_hint_seconds or 0.0)
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("invalid inferred recording duration")
    payload = b"".join(data[item.start:item.end] for item in fields if item.id != DURATION)
    payload += b"\x44\x89\x88" + struct.pack(">d", duration * 1_000_000_000 / scale)
    new_info = b"\x15\x49\xa9\x66" + _size(len(payload)) + payload
    return data[:info.start] + new_info + data[info.end:], duration


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, help="defaults to replacing the input after validation")
    parser.add_argument("--duration", type=float, help="minimum replay length in seconds")
    args = parser.parse_args()
    try:
        finalized, duration = finalize_webm(args.input.read_bytes(), args.duration)
        output = args.output or args.input
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_bytes(finalized)
        temporary.replace(output)
    except (OSError, ValueError, OverflowError) as error:
        parser.exit(1, f"unable to finalize video: {error}\n")
    print(f"Finalized {output}: {duration:.3f} seconds; encoded frames unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
