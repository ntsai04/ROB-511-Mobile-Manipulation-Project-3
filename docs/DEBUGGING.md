# Running Project 3 debugging tools

From this project root in the practice container:

```sh
make build
make run                 # terminal 1: graded runtime on TCP 9095
make test                # terminal 2: exactly 25 pass/fail tests
make visualizer          # terminal 3: HTTP 8003
```

Open `http://localhost:8003` in your local browser. In VS Code Dev Containers, forward port 8003 to your machine (Ports panel → Forward a Port). The visualizer itself runs in the same container as the runtime and bridges HTTP to TCP 9095. The bridge binds loopback by default; it can also be launched explicitly with `python3 debug_visualizer/server.py --port 8003` and the host/target options shown by `--help`. `VIS_PORT=8013 make visualizer` changes the HTTP port. Run one project's runtime at a time because every project uses TCP 9095.

The visualizer contains its own example URDF. Click **Load robot into runtime** to set `robot_description`; this deliberately replaces the active robot. It displays the runtime's acceptance/rejection status and loaded version. **Fetch current** reads the actual parameter without loading the example. Joint sliders generate `/joint_states` when **Publish joint states** is clicked; root XYZ and roll/pitch/yaw generate `/global_pose` with **Publish global pose**. A state replaces all joint positions. This tool is intended for `make run`; optional `make demo` servo/FSM nodes may overwrite manually published values.

The frame plot uses actual `/xform_world` matrices and `/tf` parent/child relationships, with no remote libraries. Drag to orbit, scroll to zoom and click **Fit view** after moving far away. Red/green/blue arrows represent X/Y/Z. Select a link to inspect its world matrix and incoming local transform; a root has no incoming edge. **Capture frames** refreshes the current topics. The raw response panel shows service statuses and messages, including timeouts. Displayed frames remain the last captured data after a failure; the connection status indicates the error. No expected transform is substituted for missing runtime output.

To run one test:

```sh
make test TESTS=test_project3.py::test_14_prismatic_axis_length_sign_and_mounted_direction
python3 tests/run_tests.py --list
```

The course runner is unchanged. Each test itself emits no stdout/stderr on pass **or failure**; the runner still prints the test name and PASS/FAIL as designed by the kit. Numerical and timeout exceptions become exit status 1. The runtime must already be running. The tests only import standard-library helpers and use the public TCP interface; they never import `src/`.

`tests/` must accompany the source into the challenge. Include `debug_visualizer/`, this Makefile and the note sheet in a local practice copy as desired; the exam's submission/visualizer rules still determine which extras are available. Stop the runtime before invoking the exam's `autograde`, since it also uses TCP 9095.
