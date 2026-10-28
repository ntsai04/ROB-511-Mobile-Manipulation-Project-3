#!/usr/bin/env python3
"""Run your own pass/fail tests: `make test`, or `python3 tests/run_tests.py`.

Every file in tests/ named test_<something> with one of these extensions is
one test, named by its file:

    .py   run with python3
    .sh   run with bash
    .c    compiled with cc  (C17), then run
    .cpp  compiled with c++ (C++20), then run; .cc works too
    .rs   compiled with rustc (standard library only), then run

A test passes when it exits with status 0 and fails otherwise, including
when it does not compile or runs longer than the time limit.

A .py file may instead define top-level functions named test_<something>.
Then each function is one test, named file.py::function: it runs in its own
process (after the file's top-level code) and passes when it returns without
raising, so a failed assert fails it.

At most 25 tests are allowed, counting each such function. Tests run one at
a time, from the project root, in name order. Start `make run` in another
terminal first if your tests talk to your runtime on 127.0.0.1:9095.

    python3 tests/run_tests.py                          run every test
    python3 tests/run_tests.py test_heap.py             run the tests in these files
    python3 tests/run_tests.py test_pid.py::test_gains  run only this function
    python3 tests/run_tests.py -v                       also show passing tests' output
    python3 tests/run_tests.py --timeout 60             seconds per test (default 30)
    python3 tests/run_tests.py --list                   list the tests, run nothing
    python3 tests/run_tests.py --root DIR               run DIR/tests instead of this folder

This file is course-supplied.
"""

from __future__ import annotations

import argparse
import ast
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

MAX_TESTS = 25
RUNTIME = ("127.0.0.1", 9095)
COMPILE_TIMEOUT = 120
OUTPUT_TAIL = 60
SUFFIXES = (".py", ".sh", ".c", ".cpp", ".cc", ".rs")
# Runs one test function: the file's top-level code first (not its
# `if __name__ == "__main__":` block), with tests/ importable.
CALL_FUNCTION = (
    "import os, runpy, sys\n"
    "sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[1])))\n"
    "runpy.run_path(sys.argv[1], run_name='autorob_test')[sys.argv[2]]()\n"
)


class Test:
    def __init__(self, source: Path, function: str | None = None) -> None:
        self.source = source
        self.function = function
        self.name = f"{source.name}::{function}" if function else source.name


def test_functions(source: Path) -> list[str]:
    """Top-level test_* functions of a .py file, read without running it."""
    try:
        tree = ast.parse(source.read_bytes(), str(source))
    except (SyntaxError, ValueError):
        return []  # it fails when run, as one test
    return [node.name for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name.startswith("test_")]


def discover(tests_dir: Path) -> list[Test]:
    tests = []
    for source in sorted(tests_dir.iterdir()):
        if not (source.is_file() and source.name.startswith("test_") and source.suffix in SUFFIXES):
            continue
        functions = test_functions(source) if source.suffix == ".py" else []
        tests.extend([Test(source, name) for name in functions] or [Test(source)])
    return tests


def select(tests: list[Test], names: list[str]) -> list[Test]:
    chosen = []
    for name in names:
        name = name.split("/")[-1]
        matches = [test for test in tests if name in (test.name, test.source.name)]
        if not matches:
            sys.exit(f"No test named {name} in tests/ (see --list).")
        chosen.extend(test for test in matches if test not in chosen)
    return chosen


def runtime_listening() -> bool:
    try:
        socket.create_connection(RUNTIME, timeout=1).close()
        return True
    except OSError:
        return False


def compile_command(source: Path, binary: Path) -> list[str] | None:
    """The command that builds a compiled test, or None if it needs none."""
    suffix = source.suffix
    if suffix == ".c":
        command = ["cc", "-std=c17", "-Wall", "-Wextra", "-O1", "-g", "-D_POSIX_C_SOURCE=200809L",
                   "-o", str(binary), str(source), "-pthread", "-lm"]
        if "cJSON" in source.read_text(encoding="utf-8", errors="replace"):
            command.append("-lcjson")
        return command
    if suffix in (".cpp", ".cc"):
        return ["c++", "-std=c++20", "-Wall", "-Wextra", "-O1", "-g", "-o", str(binary), str(source), "-pthread"]
    if suffix == ".rs":
        return ["rustc", "--edition", "2021", "-O", "-o", str(binary), str(source)]
    return None


def run_command(test: Test, binary: Path) -> list[str]:
    if test.function:
        return [sys.executable, "-c", CALL_FUNCTION, str(test.source), test.function]
    if test.source.suffix == ".py":
        return [sys.executable, str(test.source)]
    if test.source.suffix == ".sh":
        return ["bash", str(test.source)]
    return [str(binary)]


def kill_group(process: subprocess.Popen) -> None:
    """Stop the test and anything it started."""
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    process.wait()


def run_one(test: Test, root: Path, build_dir: Path, timeout: float) -> tuple[bool, str, str]:
    """(passed, short reason, output) for one test."""
    binary = build_dir / test.source.name.replace(".", "_")  # test_a.c and test_a.cpp may both exist
    build = compile_command(test.source, binary)
    if build is not None:
        build_dir.mkdir(exist_ok=True)
        if shutil.which(build[0]) is None:
            return False, f"{build[0]} is not installed here", ""
        try:
            compiled = subprocess.run(build, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      timeout=COMPILE_TIMEOUT)
        except subprocess.TimeoutExpired:
            return False, "did not compile in time", ""
        if compiled.returncode != 0:
            return False, "does not compile", compiled.stdout.decode("utf-8", "replace")
    process = subprocess.Popen(run_command(test, binary), cwd=root, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        kill_group(process)
        return False, f"timed out after {timeout:g} s", ""
    kill_group(process)  # anything the test left running in the background
    text = output.decode("utf-8", "replace")
    if process.returncode == 0:
        return True, "", text
    return False, f"exit status {process.returncode}", text


def show(output: str) -> None:
    lines = output.rstrip("\n").splitlines()
    if len(lines) > OUTPUT_TAIL:
        print(f"    ... ({len(lines) - OUTPUT_TAIL} earlier lines not shown)")
        lines = lines[-OUTPUT_TAIL:]
    for line in lines:
        print(f"    {line}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run your own tests in tests/.")
    parser.add_argument("names", nargs="*", help="test files or file.py::function names (default: all)")
    parser.add_argument("-v", "--verbose", action="store_true", help="also show passing tests' output")
    parser.add_argument("--timeout", type=float, default=30.0, help="seconds per test (default 30)")
    parser.add_argument("--list", action="store_true", help="list the tests and run nothing")
    parser.add_argument("--root", type=Path, help="project root whose tests/ to run (default: this file's project)")
    arguments = parser.parse_args()
    root = (arguments.root or Path(__file__).resolve().parent.parent).resolve()
    tests_dir = root / "tests"
    if not tests_dir.is_dir():
        print(f"No tests/ folder in {root}.")
        return 1

    every_test = discover(tests_dir)
    if arguments.list:
        for test in every_test:
            print(test.name)
        return 0
    if len(every_test) > MAX_TESTS:
        print(f"tests/ holds {len(every_test)} tests; at most {MAX_TESTS} are allowed. Remove or merge some.")
        return 2
    tests = select(every_test, arguments.names) if arguments.names else every_test
    if not tests:
        print("No tests found. Add files named tests/test_<name>.py (or .sh, .c, .cpp, .rs).")
        return 1
    if not runtime_listening():
        print("Note: nothing is listening on 127.0.0.1:9095. Tests that talk to your runtime need "
              "`make run` running in another terminal.")

    passed = 0
    for test in tests:
        started = time.monotonic()
        ok, reason, output = run_one(test, root, tests_dir / ".build", arguments.timeout)
        elapsed = time.monotonic() - started
        if ok:
            passed += 1
            print(f"PASS  {test.name}  ({elapsed:.1f} s)")
        else:
            print(f"FAIL  {test.name}  ({reason})")
        if output.strip() and (not ok or arguments.verbose):
            show(output)
    print(f"{passed} of {len(tests)} tests passed")
    return 0 if passed == len(tests) else 1


if __name__ == "__main__":
    sys.exit(main())
