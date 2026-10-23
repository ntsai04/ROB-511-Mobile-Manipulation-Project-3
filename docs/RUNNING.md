# Building, running, and recording Project 3

The [project portfolio](../README.md) describes Luma and its choreography. This
guide covers the runtime, local viewer, recording tools, and submission workflow.

A complete Python robot stack, plus an offline viewer and portfolio demo. Only
Python 3.10+ standard-library modules are required. The runtime reuses Project
1's pub/sub hub and TCP/JSON gateway with Project 2's in-process node interfaces.
URDF interpretation, quaternion math, transform composition, and tree traversal
are implemented here without kinematics or URDF-model libraries.

For a source reading order, a comparison with Projects 1 and 2, and the state
and frame conventions behind the algorithms, see
[the code walkthrough](CODE_WALKTHROUGH.md). The runtime modules also contain
detailed teaching comments; their named calculation steps remain readable when
comments are stripped. The walkthrough adds no mutation tests or worksheet solutions.

## Build and run

From this project directory, with GNU Make:

```sh
make build
make run
```

`make run` stays in the foreground at **127.0.0.1:9095**, speaking newline-delimited
TCP/JSON (not HTTP or WebSocket). It starts the gateway, parameter server, robot
state publisher, and world state publisher, with **no robot loaded**. It does not
start anything that publishes joint states, trajectories, or global poses.

Load any valid flat URDF through `/param_server/set_param` with name
`robot_description`, then poll `/robot_state_publisher/description_status` through
`get_param` until its version reaches the version returned by `set_param`.
Subscribe to `/tf` or `/xform_world` to receive complete snapshots at 10 Hz.

The final runtime includes:

- Versioned arbitrary JSON parameters, description validation, transactional
  replacement, and rejection that preserves the previous robot and joint state.
- Revolute/continuous axis-angle rotation and prismatic translation composed
  after each joint origin, using unit quaternions. Missing joints in each new
  `/joint_states` message return to zero; fixed and unknown names are ignored.
- Exact incoming joint-state stamps on every transform. FK never clamps limits.
- Persistent `/global_pose`, composed into every link's pose, including the root.
- Root discovery and iterative tree traversal using only the latest complete
  `/tf` message. World matrices are column-major and remain z-up.
- Complete periodic messages for late subscribers and TCP lines up to 4 MiB.

The authoritative wire contracts are `spec/FK_API.md`, `spec/ROBOT_DESCRIPTION.md`,
and `spec/ROSBRIDGE_PROTOCOL.md`.

## Demo, viewer, and portfolio

In separate terminals:

```sh
make demo       # the full runtime plus servo, FSM, bundled robot and choreography
make viewer     # external TCP client and local HTTP viewer on port 8080
```

Open **http://127.0.0.1:8080/viewer/**. Drag to orbit, scroll to zoom, or use the
pause/resume and reset controls. The browser receives `/xform_world` through a
small HTTP/SSE adapter; the robot runtime itself keeps the required TCP protocol.
The canvas renderer draws URDF box/cylinder/sphere visuals and their local origins
from the supplied world matrices. It needs no graphics packages or network assets.
Robots whose visuals require external meshes can be inspected as a frame skeleton.

The custom robot **Luma** has six movable joints and twelve links. Its eight-state
sequence wakes, looks left, waves, reaches, closes the gripper, presents, and bows.
Targets and transitions live in `examples/choreography.json`; the primitive-only
robot is `examples/showcase.urdf`. The servo uses a stable exponential proportional
step with gain 2.5/s and applies limits. The optional FSM `hold_seconds` field holds
a state after arrival so the motion is easy to follow. A loop lasts about 20 seconds.

The root README is the main GitHub Pages portfolio. A separate designed page at
**http://127.0.0.1:8080/portfolio/** contains the 24-second preview, an interactive
replay of real captured runtime traffic, and an explanation of the robot,
choreography, and rendering. The root README reserves a link for a screen
recording of the live dance.

To make a new recording while the demo is running:

```sh
make record
```

This captures 24 seconds to `portfolio/demo_recording.json`. Open
`http://127.0.0.1:8080/viewer/?replay=../portfolio/demo_recording.json`, then choose
**Export video**. After the full replay, the local server saves
`portfolio/demo.webm`. **Save still frame** saves `portfolio/poster.png`. On a
static host these buttons offer downloads instead. `make portfolio` packages the
separate designed page, viewer, recording, video, and poster as a standalone
static site; see `portfolio/README.md` for the directory layout. Keep the existing
README-based Pages entry point when using the README as your portfolio page.

## Demo services and saved choreography

Only `make demo` exposes the joint servo and FSM services. The servo consumes the
last `/joint_trajectory` point, remembers targets by name across robot reloads,
and provides `/joint_state_publisher/reset`. The FSM publishes a target on entry
and changes state once all named joints arrive within epsilon.

| Service | Purpose |
| --- | --- |
| `/fsm/update_logic` | Validate and atomically deploy a table, enter its initial state, and run. |
| `/fsm/get_logic` | Read the deployed table. |
| `/fsm/set_running` | Pause, resume, or query running state. |
| `/fsm/fire_transition` | Take a named transition while running. |
| `/fsm/save_to_file` | Save a draft under `fsm/<identifier>.json` without deploying it. |
| `/fsm/load_from_file` | Read a saved draft without deploying it. |
| `/fsm/list_saved` | List saved identifiers in sorted order. |

File identifiers accept only 1-64 letters, digits, underscores, and hyphens. See
the spec for JSON request shapes. Pausing stops FSM transitions; the servo still
finishes its current target. Resuming reissues the current state's target.

## Verify and submit

Stop any running demo/runtime before testing, since the integration suite owns
port 9095:

```sh
make clean
make build
make test
make submission
```

The tests exercise the public TCP boundary and analytic 3D expectations:
checkpoint validation/rollback, real XML, Fetch, near-1 MiB messages, joint motion,
world poses, timestamps, reloads, continuous publishing, late subscribers, and the
servo/FSM/persistence behavior. The submission archive has its Makefile directly
at the root and includes the offline source, examples, and portfolio assets.
`make clean` removes Python caches and generated ZIP/tar packages while preserving
the portfolio video, poster, telemetry, and saved choreographies.

On Windows without GNU Make, use the matching commands:

```powershell
python -m compileall -q src tools tests
python src/main.py
python src/main.py --demo
python tools/viewer_server.py
python tools/record_demo.py --duration 24 --output portfolio/demo_recording.json
python -m unittest discover -s tests -v
python tools/package_submission.py
python tools/package_portfolio.py
```

With Make and a different interpreter name, use `make PYTHON=python <target>`.
`bash make_submission.sh python` also creates the submission archive.

Fetch robot description © Fetch Robotics, Inc., CC BY-NC-SA 4.0. The Fetch URDF is
unmodified and its license remains in `robots/fetch/LICENSE.md`. The showcased
Luma robot is original primitive geometry; it uses no Fetch meshes or description.
