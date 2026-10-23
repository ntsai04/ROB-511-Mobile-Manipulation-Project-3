"""Check recorder finalization without a browser, video codec or transcoder."""

from pathlib import Path
from email.message import Message
import io
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from webm_duration import (
    CLUSTER, DURATION, INFO, SEGMENT, TRACKS, _children, _element, _size, finalize_webm,
)
import viewer_server


def element(identifier, payload=b"", unknown=False):
    tag = identifier.to_bytes((identifier.bit_length() + 7) // 8, "big")
    return tag + (b"\x01\xff\xff\xff\xff\xff\xff\xff" if unknown else _size(len(payload))) + payload


def recording(scale=1_000_000, duration=None, group=False, muxer=b"Chrome", codec=b"V_VP9",
              extra_top=b"", extra_cluster=b"", unknown_segment=True):
    header = element(0x1A45DFA3, element(0x4282, b"webm"))
    info = element(0x2AD7B1, scale.to_bytes(3, "big"))
    info += element(0x4D80, muxer) + element(0x5741, muxer)
    if duration is not None:
        info += element(DURATION, struct.pack(">f", duration))
    track = element(0xD7, b"\x01") + element(0x83, b"\x01") + element(0x86, codec)
    content = element(INFO, info) + element(TRACKS, element(0xAE, track))
    for base in (0, 1000):
        blocks = element(0xE7, base.to_bytes(2, "big"))
        for relative in (0, 40, 80):
            frame = b"\x81" + relative.to_bytes(2, "big", signed=True) + b"\x80encoded-frame\x1c\x53\xbb\x6b"
            if group:
                additions = element(0x75A1, element(0xA6, element(0xEE, b"\x01") + element(0xA5, b"extra-frame")))
                blocks += element(0xA0, element(0xA1, frame) + element(0xFB, b"\xff") + additions)
            else:
                blocks += element(0xA3, frame)
        content += element(CLUSTER, blocks + extra_cluster, unknown=True)
    return header + element(SEGMENT, content + extra_top, unknown=unknown_segment)


def info_element(data):
    header = _element(data, 0, len(data))
    segment = _element(data, header.end, len(data))
    return _element(data, segment.data_start, segment.end)


class DurationTests(unittest.TestCase):
    def test_inferred_frame_end_and_encoded_bytes_are_preserved(self):
        for group in (False, True):
            original = recording(group=group)
            finalized, seconds = finalize_webm(original)
            self.assertAlmostEqual(seconds, 1.12)
            old_info, new_info = info_element(original), info_element(finalized)
            self.assertEqual(original[:old_info.start], finalized[:new_info.start])
            # Covers all Tracks and every encoded frame, including ID-like data
            # within a frame that must never be mistaken for a Cues element.
            self.assertEqual(original[old_info.end:], finalized[new_info.end:])
            durations = [item for item in _children(finalized, new_info) if item.id == DURATION]
            self.assertEqual(len(durations), 1)
            self.assertEqual(durations[0].end - durations[0].data_start, 8)
            self.assertAlmostEqual(struct.unpack(">d", finalized[durations[0].data_start:durations[0].end])[0], 1120)
            self.assertEqual(finalize_webm(finalized)[0], finalized)

    def test_replay_hint_and_timestamp_scale_use_segment_ticks(self):
        finalized, seconds = finalize_webm(recording(scale=2_000_000, duration=999), 3.0)
        self.assertEqual(seconds, 3.0)
        item = next(item for item in _children(finalized, info_element(finalized)) if item.id == DURATION)
        self.assertEqual(struct.unpack(">d", finalized[item.data_start:item.end])[0], 1500)

    def test_refuses_offsets_indexing_unfamiliar_muxers_or_truncation(self):
        malformed = [
            recording(extra_top=element(0x1C53BB6B)),  # Cues after an unknown-size Cluster
            recording(extra_cluster=element(0xA7, b"\x01")),  # Cluster Position
            recording(unknown_segment=False),
            recording(muxer=b"other-muxer"),
            recording(codec=b"A_OPUS"),
            recording()[:-2],
            b"",
        ]
        raw = recording()
        info = info_element(raw)
        malformed.append(raw[:info.start] + element(0x114D9B74) + raw[info.start:])
        for data in malformed:
            with self.subTest(prefix=data[:20]), self.assertRaises(ValueError):
                finalize_webm(data)

    def test_invalid_duration_never_changes_input(self):
        data = recording()
        for hint in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                finalize_webm(data, hint)
        self.assertEqual(data, recording())


class VideoUpload(viewer_server.ViewerHandler):
    """Exercise the HTTP handler body using streams, without binding a port."""

    def __init__(self, data):
        self.path = "/api/video"
        self.headers = Message()
        self.headers["Content-Length"] = str(len(data))
        self.headers["X-Video-Duration"] = "3"
        self.headers["Origin"] = "http://127.0.0.1:8080"
        self.server = SimpleNamespace(server_port=8080)
        self.rfile = io.BytesIO(data)
        self.response = None

    def json_response(self, value, status=200):
        self.response = status, value

    def send_error(self, status, message=None):
        self.response = status, {"status": message}


class UploadTests(unittest.TestCase):
    def test_uploaded_video_is_finalized_and_invalid_upload_preserves_saved_video(self):
        project = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix="video-upload-", dir=project / "tests") as directory:
            with patch.object(viewer_server, "ROOT", Path(directory)):
                upload = VideoUpload(recording(group=True))
                upload.do_POST()
                self.assertEqual(upload.response[0], 200)
                self.assertEqual(upload.response[1]["duration"], 3.0)
                output = Path(directory) / "portfolio" / "demo.webm"
                saved = output.read_bytes()
                self.assertEqual(finalize_webm(saved, 3.0)[0], saved)
                invalid = VideoUpload(recording(extra_top=element(0x1C53BB6B)))
                invalid.do_POST()
                self.assertEqual(invalid.response[0], 400)
                self.assertEqual(output.read_bytes(), saved)
                self.assertFalse(output.with_suffix(".webm.tmp").exists())


if __name__ == "__main__":
    unittest.main()
