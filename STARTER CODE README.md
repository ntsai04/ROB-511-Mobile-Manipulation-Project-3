# Project 3 starter directories

This folder contains **four alternatives**, not one submission. Choose one: `python/`, `c/`,
`cpp/`, or `rust/`. Read `docs/PROJECT3_FK.md` first; it is the full assignment.

Each starter builds and runs as-is, but it does nothing useful yet. It gives you:

| Language | Where to start | Course-supplied XML parser |
|---|---|---|
| Python | `src/main.py`; `src/urdf_example.py` shows the XML plumbing | the standard library's `xml.etree.ElementTree` |
| C | `src/main.c`; `src/urdf_example.c` shows the XML plumbing | ezxml 0.8.6 in `third_party/ezxml/` (built by the `Makefile`) |
| C++ | `src/main.cpp`; `src/urdf_example.cpp` shows the XML plumbing | pugixml 1.14 in `third_party/pugixml/` |
| Rust | `src/main.rs` (`print_links` shows the XML plumbing) | roxmltree 0.20.0 in `vendor/roxmltree/` (a path dependency in `Cargo.toml`) |

The examples only read XML. Deciding what the elements mean, which descriptions are valid, and
the forward kinematics are your job. The vendored parsers are unmodified upstream releases;
their licenses are next to their sources. You MAY replace them with your own parser.

What the starters do **not** include is your middleware or any node. Bring your Project 1
rosbridge gateway and publish/subscribe layer, write the nodes `spec/FK_API.md` describes, and
extend the `Makefile` so that `make build` compiles everything, `make run` starts the graded
runtime (gateway on `127.0.0.1:9095`, `param_server`, `robot_state_publisher`,
`robot_world_state_publisher`), and `make demo` adds your `joint_state_publisher` and
`finite_state_machine`. How you split this into files, classes, and processes is up to you.
Remember the line-size requirement in the handout: your gateway and every node-to-node
connection must accept lines of at least 1 MiB.

To submit, create `submission.tar.gz` from the **contents** of the one directory you chose, so
its `Makefile` is at the archive root. From the student kit's root, `./make_submission.sh
<python|c|cpp|rust>` does this for you. Do not submit this parent `starter/` directory.
