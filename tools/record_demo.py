"""Capture actual runtime topic traffic for the portable portfolio replay."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from wire_client import TOPICS, WireClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--output", type=Path, default=Path("portfolio/demo_recording.json"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9095)
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error("duration must be in (0, 600] seconds")
    with WireClient(args.host, args.port) as client:
        description = client.call("/param_server/get_param", {"name": "robot_description"})
        if not description.get("result") or not isinstance(description["values"]["value"], str):
            raise SystemExit("Load a robot first, or start python src/main.py --demo")
        for topic, type_name in TOPICS.items():
            client.subscribe(topic, type_name)
        start = time.monotonic()
        frames = []
        while time.monotonic() - start < args.duration:
            try:
                message = client.next_publication(min(0.5, args.duration - (time.monotonic() - start)))
            except TimeoutError:
                continue
            frames.append({"t": round(time.monotonic() - start, 6),
                           "topic": message["topic"], "msg": message["msg"]})
    if not any(frame["topic"] == "/xform_world" for frame in frames):
        raise SystemExit("No /xform_world messages received; recording was not saved")
    result = {"format": "autorob-recording-v1", "recorded_at": datetime.now(timezone.utc).isoformat(),
              "duration": args.duration, "robot_description": description["values"]["value"],
              "frames": frames}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    print(f"Recorded {len(frames)} actual topic messages over {args.duration:g}s to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
