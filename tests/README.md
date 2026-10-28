# Your own tests

You may submit up to **25 pass/fail tests of your own** with your project. They are not graded.
They are a debugging tool: in the mutation challenge they come with your project into your
container, so you can run them there while you hunt for the bug. Comments in them are removed
there, as in the rest of your code.

This folder already has everything you need:

| File | What it is |
|---|---|
| `run_tests.py` | Runs your tests. `make test` calls it. Course-supplied. |
| `rosbridge_client.py` | A small Python client for the course TCP/JSON protocol, for your Python tests. Course-supplied; you may change it. |
| `test_*_example.py` | An example test. Copy it to start your own. It fails until your runtime implements what it checks. |
| `README.md` | This file. |

## Adding tests to a project you already have

Copy this whole `tests/` folder into your project's root, next to your `Makefile`, and add
the `test` target from rule 1 below to your `Makefile`. Then write your tests (rename or delete
the example).

## Running your tests

Your runtime has to be running for any test that talks to it:

```bash
make run                                # terminal 1: leave it running
make test                               # terminal 2: run every test
make test TESTS=test_heap_sort.py       # run only the named tests (file or file.py::function)
python3 tests/run_tests.py -v           # also show the output of tests that pass
```

The output looks like this, followed by each failing test's own output:

```text
PASS  test_heap_sort.py  (0.2 s)
FAIL  test_plan_detour.py  (exit status 1)
1 of 2 tests passed
```

`make test` exits with status 0 only when every test passed. In the mutation challenge,
stop `make run` before you run `autograde`: both use port 9095.

## Rules

Follow these so your submission is accepted with your tests in it.

1. **Folder.** Tests live in `tests/` at the top of your project, next to your `Makefile`.
   `make_submission.sh` includes it; if you build `submission.tar.gz` yourself, include it
   too. Keep `run_tests.py` there and keep the `test` target in your `Makefile`:

   ```make
   test:
   	@python3 tests/run_tests.py $(TESTS)
   ```

2. **One file per test**, named `tests/test_<name>.<ext>`; the name is what `make test`
   reports, so make it say what the test checks (`test_pid_controller_works.py`). A Python file
   may instead define functions named `test_<name>`: then each function is one test, reported
   as `test_file.py::test_name`. Other files in `tests/` (helpers, data) must not start with
   `test_`.
3. **At most 25 tests**, counting each test function. `run_tests.py` refuses to run more;
   `python3 tests/run_tests.py --list` shows how it counts yours.
4. **Languages.** Use only what the course environment has. There is no network and nothing can
   be installed.

   | Extension | How it runs | You may use |
   |---|---|---|
   | `.py` | `python3 tests/test_x.py` | the Python 3 standard library |
   | `.sh` | `bash tests/test_x.sh` | bash and standard Unix tools |
   | `.c` | compiled with `cc -std=c17`, then run | C17, POSIX, pthreads, cJSON |
   | `.cpp`, `.cc` | compiled with `c++ -std=c++20`, then run | C++20, POSIX, threads, nlohmann JSON |
   | `.rs` | compiled with `rustc` as one file, then run | the Rust standard library only (no crates) |

   Your tests do not have to be in the language of your project: a C++ project may have Python
   tests.
5. **Pass or fail.** A test passes when it exits with status 0. Anything else fails it,
   including a compile error or running longer than 30 seconds. Tests run one at a time from
   your project root, in name order. Anything a test leaves running is stopped when it ends.
6. **Keep tests out of your build.** `make build` and `make run` must not compile or depend on
   anything in `tests/`. In particular, do not let a wildcard such as `$(wildcard */*.c)` pick up
   test files.
7. **Plain, complete source files.** Save tests as UTF-8 text. Each `.py`, `.c`, `.cpp`, `.cc`,
   `.h`, `.hpp` and `.rs` file in `tests/` must be complete: Python must parse, and comments and
   string literals in C, C++ and Rust must be closed. If one cannot be processed, your project
   cannot be used in the mutation challenge until you fix that file and submit again.
8. **Small.** No binaries, no build output, no large data. `run_tests.py` builds compiled tests
   into `tests/.build/`, which `make_submission.sh` leaves out; leave it out too if you build
   your archive yourself.
9. **Rust projects:** do not add `[dev-dependencies]` to your `Cargo.toml` for your tests. The
   course environment builds offline from a fixed set of crates, and a crate outside that set
   would stop your project from building.

## Writing a test

**Python**, with the supplied client (see `test_*_example.py`):

```python
from rosbridge_client import Rosbridge

with Rosbridge() as ros:
    reply = ros.call_service("/heap_sort", {"numbers": [3, -1, 2.5]})

assert reply["result"] is True, reply
assert reply["values"]["sorted"] == [-1, 2.5, 3], reply
```

A failed `assert` (or any other error) fails the test, and its message is shown, so put what
you received in the message. Several tests can share one file as functions; each runs on its
own, after the file's top-level code:

```python
from rosbridge_client import Rosbridge

def test_heap_sort_sorts_negatives():
    with Rosbridge() as ros:
        reply = ros.call_service("/heap_sort", {"numbers": [2, -5, 0]})
    assert reply["values"]["sorted"] == [-5, 0, 2], reply

def test_heap_sort_accepts_empty_list():
    with Rosbridge() as ros:
        reply = ros.call_service("/heap_sort", {"numbers": []})
    assert reply["result"] is True and reply["values"]["sorted"] == [], reply
```

Comments in your tests are removed in the mutation challenge, like the rest of your code, so
let test names and assert messages say what a test checks. The client also publishes and subscribes:

```python
ros.advertise("/map", "nav_msgs/OccupancyGrid")
ros.publish("/map", grid)
ros.subscribe("/path", "nav_msgs/Path")               # waits until it is registered
plan = ros.next_message("/path", timeout=5)          # the next message on /path
```

Every wait has a timeout and raises `TimeoutError` instead of hanging. Messages are plain
dicts, exactly as in the course's rosbridge protocol document.

**C, C++ or Rust:** connect a TCP socket to `127.0.0.1:9095`, send one JSON object per line,
read lines back, and `return 0` from `main` to pass or `return 1` to fail. Set a receive
timeout so a missing reply fails the test instead of hanging it.

**Testing your own code directly:** a test does not have to use the runtime. A C or C++ test
may `#include "../src/heap.c"` (or your header and source) and check your functions directly,
and a Python test may import your modules after adding your source folder to `sys.path`.

**Shell:** bash can open a TCP connection itself:

```bash
exec 3<>/dev/tcp/127.0.0.1/9095
printf '%s\n' '{"op":"call_service","service":"/heap_sort","id":"1","args":{"numbers":[2,1]}}' >&3
IFS= read -r -t 5 reply <&3 && [[ $reply == *'"result":true'* || $reply == *'"result": true'* ]]
```
