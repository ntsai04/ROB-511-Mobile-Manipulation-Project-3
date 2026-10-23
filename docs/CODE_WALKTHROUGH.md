# Reading the three projects

This is a reading map for the existing implementations. Projects 1 and 2 are
unchanged. Project 3 keeps their transport design and makes its robot algorithms
explicit. No new tests or worksheet solutions are included here.

## The common structure

Read these names once, then look for the same names in all three projects:

| Component | What it owns | What to follow |
| --- | --- | --- |
| `ClientConnection` | One socket, one outgoing queue, registrations for that connection | `send` queues a dictionary; `_write_loop` is the only socket writer. |
| `RosbridgeServer` | Request dispatch and delivery | `handle_request` selects the operation; `_call_service` and `_publish` are the two main routes. |
| `Hub` | Subscribers, service providers, outstanding calls | Subscribe by topic; discover providers by service; match responses by connection and generated call ID. |
| A domain node | Algorithm state and public service/topic handlers | Validate an input, change its own state, and publish or return the result. |
| `main.py` | Node construction, registration, update order, socket accept loop | Read the registration calls to reconstruct the node graph. |

Project 1 calls its heap and A* services directly from the gateway. Project 2
introduces `local_services`, `topic_listeners`, and `_register_services`. Project
3 uses that same Project 2 interface. Its injected `call_service` and `publish`
arguments are ordinary functions; they are not another socket implementation.

All local service handlers return `(result, values, status)`. The gateway builds
the external envelope containing `op`, `service`, `id`, and those three fields.
A local handler does not need to understand caller IDs or TCP framing.

A socket reader may receive input while the update thread is running. Locks
protect related state as one snapshot. Network writes happen through per-client
queues after releasing algorithm and routing locks. Holding a lock while waiting
for a slow client would couple otherwise unrelated operations.

## Project 1: revisit the submitted algorithms

These files are in the sibling `ROB-511-Mobile-Manipulation-Project-1` folder.

| File / functions | Main idea to explain from the code |
| --- | --- |
| `src/heap.py`: child-index helpers, `sift_down`, `make_min_heap`, `extract_min`, `heap_sort` | Array indices represent a binary tree. `sift_down` repairs the root while the child subtrees already satisfy the min-heap property. Extraction repairs the heap after replacing its root. |
| `src/heap_services.py` | Converts service arguments into algorithm inputs and returns the common service triple. |
| `src/astar_node.py`: grid helpers | Keep row-major indexing, map bounds, occupancy rules, and cell/world conversion separate from search. |
| `astar`, `next_from_frontier` | Follow the frontier, best known path cost, Manhattan heuristic, and predecessor records. Frontier selection calls the public heap-sort service. |
| `reconstruct_path`, `path_message` | Follow predecessors from the goal, then restore start-to-goal order and turn cells into message poses. |

The useful questions are which values are indices, which are grid coordinates,
which are world coordinates, and which records describe a best known path. A
comparison can be correct for one of these meanings and wrong for another.

## Project 2: revisit the submitted dynamics and control

These files are in the sibling `ROB-511-Mobile-Manipulation-Project-2` folder.

| File / functions | Main idea to explain from the code |
| --- | --- |
| `src/main.py`, `ArmSimNode.step` | Trace the controller effort into the dynamics and selected integration step, then into published joint state. |
| `src/arm_lib/pid.py`: `effort`, `reset` | Position and velocity errors are setpoint minus actual. The integral accumulates position error times elapsed time and has its own bounded state. Reset preserves the number of joints. |
| `src/arm_lib/integrators.py`: `euler`, `midpoint`, `verlet`, `rk4` | The coupled state is `(q, qdot)` and its derivative is `(qdot, acceleration(t,q,qdot))`. Every stage has a specific trial time and trial state. |
| `src/arm_lib/arm_dynamics.py` | Produces acceleration from the robot state and applied torque; this is distinct from choosing how to step time. |
| `src/kinematics.py` | Planar FK accumulates relative joint angles. IK reduces a three-link orientation target to a two-link wrist target before solving. |
| `src/ik_action_node.py`, `src/ik_trial_node.py` | Coordinate a solve/trajectory and its progress using the same service and topic interfaces. |

For integration code, keep the original state distinct from the trial states.
For control code, keep the sign of the error, the units of `dt`, and controller
memory distinct from the plant's physical state. These are the concepts that
connect the worksheet's examples to the submitted Python implementation.

## Project 3: suggested reading order

1. **`src/main.py`**: compare `_register_services` with Project 2. Read the three
   graded topic registrations, then the servo/FSM registrations inside `--demo`.
   Follow the periodic order: servo, robot state, world state, FSM.
2. **`src/param_server.py`**: each dictionary entry holds `(value, version)`.
   Versions belong to names. Storing a URDF does not mean accepting a robot.
3. **`src/transforms.py`**: read `compose_transforms` first, then the quaternion
   operations it calls. Finish with the matrix packing function.
4. **`src/robot_description.py`**: the short public parser lists its phases.
   Read one joint's parsing and `Joint.joint_transform` before the tree checks.
5. **`src/robot_state_publisher.py`**: follow `update`, `_process_description`,
   `_replace_robot`, and `joint_configuration`. Then read the input callback.
6. **`src/robot_world_state_publisher.py`**: follow `_read_tree` into
   `_compose_tree`. Each stack entry carries one link's world pose.
7. **`src/joint_state_publisher.py`**, then **`src/finite_state_machine.py`**:
   distinguish moving toward a target from choosing the next target.

The module introductions and inline comments explain Python details and the
reasons for each operation. The method names and intermediate variables also
carry those meanings when comments are absent.

## The transform calculations to understand

A transform represents a point conversion from a child frame to its parent:

```text
point_parent = rotation_parent_child * point_child + translation_parent_child
```

For parent/child transform composition, the local displacement must first be
rotated into the outer frame:

```text
translation_composed = translation_parent + rotate(q_parent, translation_child)
quaternion_composed  = q_parent * q_child
```

`quaternion_multiply(left, right)` applies the right rotation first, then the
left rotation. An identity rotation has quaternion `(0, 0, 0, 1)` in the xyzw
ordering used throughout the project. Multiplication and matrix conversion
normalize the quaternion so roundoff does not accumulate into a scaled rotation.

For URDF, the fixed origin uses `Rz(yaw) * Ry(pitch) * Rx(roll)`. A joint's local
motion follows that origin: `T_joint = T_origin * T_motion(q)`. Revolute and
continuous motion normalizes the axis; prismatic motion multiplies the axis as
written by the position. FK does not clamp positions to the limits. The separate
demo servo does clamp its targets and positions.

World composition uses `T_global_child = T_global_parent * T_parent_child`.
Each branch receives its own stack entry rather than overwriting a shared
accumulator. The root also needs an output pose. Matrix serialization is a
separate final step: indices `12..14` hold translation because the flattened
matrix stores one column at a time.

## State and replacement rules

| State | Replacement rule |
| --- | --- |
| A named parameter | Every set replaces its complete JSON value and increments that name's version. |
| The loaded robot | Validate the complete candidate first. Rejection preserves the previous robot and joint state. |
| `_processed_version` | The latest description examined, including a rejected one. |
| `_loaded_version` | The latest description accepted. It changes only on a successful replacement. |
| Joint positions and stamp | One `/joint_states` message replaces both together. Unnamed movable joints use zero. Copy the incoming stamp. |
| The world publisher's tree | One complete `/tf` message replaces the whole previous tree. Never merge old edges. |
| Global pose | Retain independently until another pose arrives, including across robot reloads. |
| Servo targets | Retain by joint name, including commands that precede a new robot model. |
| FSM logic | Validate the complete table before replacing the deployed logic; enter the initial state and issue its target. |

The robot state publisher publishes its new edges before reporting description
acceptance. That ordering makes the status meaningful: a client observing an
accepted version can then receive transforms for that robot. Periodic updates
continue publishing complete snapshots even without a new command.

## Small, intentional transport differences

Project 3 retains the same routing maps, operations, queues and service triple.
Some implementation details remain stricter than the submitted earlier code:

- The gateway reads a whole binary line up to 4 MiB, which protects the required
  large URDF requests and replies from truncation. Packets and lines differ.
- `Hub.begin_call` does provider lookup and pending-call registration under one
  lock. It combines the two familiar steps so a disconnect cannot fall between.
- Responses also match the advertised service name, and generic JSON `null`
  values stay `null` when forwarded.
- Required transport fields and finite JSON numbers are checked. One invalid
  callback or local service request does not disable unrelated connections.
- Project 3 accepts a valid publish payload even if its connection did not first
  advertise that topic. The supplied protocol does not make advertisement a
  prerequisite; ordinary clients can still advertise as in the earlier projects.

These differences do not introduce a second protocol or a different node
architecture. They are validation and bookkeeping around the familiar paths.

## What is outside the robot algorithm reading path

The grading entry point is `src/main.py` without `--demo`. The viewer, recording,
video-container metadata, and portfolio packaging code live in `viewer/`,
`portfolio/`, and `tools/`. They are presentation utilities, separate from the
graded kinematics pipeline. The starter's `src/urdf_example.py` is an unused XML
example; the actual description parser is `src/robot_description.py`.

The current development regression suite is unchanged. It is separate from any
future silent, at-most-25-test mutation submission. The course has not released
that format, so no exam runner or new mutation test suite is assumed here.
