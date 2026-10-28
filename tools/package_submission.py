#!/usr/bin/env python3
"""Package this checkout for the course container, using only the standard library."""
from __future__ import annotations

import ast
import gzip
from pathlib import Path
import tarfile


ROOT = Path(__file__).resolve().parents[1]
ENTRIES = ("Makefile", "make_submission.sh", "src", "scripts", "tools", "tests",
           "visualizer", "viewer", "maps", "examples", "robots")
SKIP = {".git", ".build", "build", "bin", "target", "__pycache__", ".DS_Store"}


def main() -> None:
    files = []
    for entry in ENTRIES:
        base = ROOT / entry
        candidates = sorted(base.rglob("*")) if base.is_dir() else [base]
        for path in candidates:
            relative = path.relative_to(ROOT)
            if any(part in SKIP for part in relative.parts) or path.suffix in (".pyc", ".pyo"):
                continue
            if path.is_symlink():
                raise SystemExit("Cannot package symbolic links: " + str(relative))
            if path.is_file():
                if not path.resolve().is_relative_to(ROOT):
                    raise SystemExit("File outside project: " + str(relative))
                files.append(path)
    tests = 0
    for path in files:
        if path.suffix == ".py":
            tree = ast.parse(path.read_bytes(), filename=str(path.relative_to(ROOT)))
            if path.parent == ROOT / "tests" and path.name.startswith("test_"):
                functions = sum(isinstance(node, ast.FunctionDef) and node.name.startswith("test_")
                                for node in tree.body)
                tests += functions or 1
        elif path.parent == ROOT / "tests" and path.name.startswith("test_") and path.suffix in (".sh", ".c", ".cpp", ".cc", ".rs"):
            tests += 1
    if tests != 25:
        raise SystemExit(f"Expected exactly 25 tests; found {tests}")
    if not (ROOT / "tests/run_tests.py").is_file():
        raise SystemExit("Missing supplied test runner")
    output = ROOT / "submission.tar.gz"
    temporary = output.with_name(output.name + ".tmp")
    try:
        with temporary.open("wb") as raw:
            with gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w") as bundle:
                    for path in sorted(set(files)):
                        name = path.relative_to(ROOT).as_posix()
                        info = bundle.gettarinfo(str(path), arcname=name)
                        info.uid = info.gid = 0
                        info.uname = info.gname = ""
                        info.mtime = 0
                        info.mode = 0o755 if path.suffix == ".sh" else 0o644
                        with path.open("rb") as source:
                            bundle.addfile(info, source)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Wrote {output.name}: {tests} tests, {len(set(files))} files, {output.stat().st_size} bytes")


if __name__ == "__main__":
    main()
