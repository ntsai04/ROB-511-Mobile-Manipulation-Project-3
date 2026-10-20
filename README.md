# Project 3 checkpoint: zero-configuration forward kinematics

This Python implementation reuses Project 1's publish/subscribe hub and TCP/JSON
gateway, including Project 2's in-process node interfaces. It uses only the
Python standard library. The three checkpoint nodes share one process and
communicate through the gateway's service and topic dispatch interfaces:

- `param_server` stores arbitrary JSON values with independent per-name versions.
- `robot_state_publisher` polls `/param_server/get_param`, validates each new
  `robot_description`, and publishes every joint's zero-configuration transform
  on `/tf`. Rejected loads leave the previous model in effect. It reports each
  processed version through `/robot_state_publisher/description_status`.
- `robot_world_state_publisher` derives the root and traverses the latest `/tf`
  tree, publishing every link's pose on `/xform_world` as a column-major matrix.

URDF interpretation, quaternion math, and transform composition are implemented
here; `xml.etree.ElementTree` handles XML syntax. The gateway reads complete
newline-delimited frames up to 4 MiB. The runtime starts without a robot loaded,
listens at `127.0.0.1:9095`, and publishes complete snapshots at 10 Hz after loading.

## Run and verify

With Python 3.10 or newer and GNU Make, from this directory:

```sh
make build
make run
```

Keep `make run` in the foreground. In another terminal, connect a TCP client and
call `/param_server/set_param` with `name: "robot_description"` and a URDF string
as `value`. Poll the description status until its `version` reaches the version
returned by `set_param`, then subscribe to `/tf` and `/xform_world` using the
protocol in `spec/ROSBRIDGE_PROTOCOL.md`. The example client in
`docs/PROJECT3_FK.md` can load `robots/fetch/fetch.urdf`.

```sh
make test         # launches a separate runtime on port 9095; stop make run first
make submission   # writes submission.tar.gz with Makefile at its root
make clean        # removes Python caches and the generated archive
```

`bash make_submission.sh python` also packages the project. On Windows without
Make, the corresponding commands are `python -m compileall -q src tools tests`,
`python src/main.py`, `python -m unittest discover -s tests -v`, and
`python tools/package_submission.py`. With Make and a different interpreter name,
use `make PYTHON=python run` (and the same override for the other targets).

Tests exercise the public TCP boundary: parameter versions and errors, real XML,
all required description rejection categories, rollback and replacement, hand-
calculated 3D chains and branches, unmodified Fetch, and large request/reply lines.

## Checkpoint scope

All joint positions are zero and the world pose is the identity. Joint-state
motion, `/global_pose`, the servo/FSM demo, viewer, and portfolio video belong to
the remaining final-project work. This checkpoint has no `make demo` target.

The authoritative contracts are `spec/FK_API.md` and
`spec/ROBOT_DESCRIPTION.md`; the original handout is `docs/PROJECT3_FK.md`.

Fetch robot description © Fetch Robotics, Inc., CC BY-NC-SA 4.0. The bundled URDF
is unmodified; its attribution and license remain in `robots/fetch/LICENSE.md`.
