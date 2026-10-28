# Project 3 — mutation debugging notes

From the project folder: `make run` (terminal 1), `make test` (terminal 2), `make visualizer` (terminal 3), then open **http://localhost:8003**. In Docker, forward port 8003 through VS Code or publish that port when creating the container. Stop `make run` before `autograde`; both use port 9095. Rerun after every fix.

Exactly **25 silent pass/fail tests**, using the supplied harness. Each function counts as one test. All tests import the unchanged Python modules directly and can run with the runtime stopped. The harness may print its standard “nothing listening” note; the test bodies never print diagnostics. To select one test: `make test TESTS=test_fk.py::test_13_prismatic_preserves_axis_length`. To list them: `python3 tests/run_tests.py --list`. Running `python3 tests/test_fk.py` alone does not invoke its test functions.

Names in the table are prefixed `test_` inside `test_fk.py`. Tolerance is 1e-9 (relative and absolute). Quaternion comparisons use equivalent rotation matrices, accepting either quaternion sign. Expected transforms use independent row-matrix multiplication, explicit RPY matrices and Rodrigues rotations rather than the implementation's quaternion/composition functions.

| ID / function suffix | Probe and required invariant | Inspect first |
|---|---|---|
| 01 `quaternion_normalization` | Mixed-sign quaternion normalized by its norm; huge inputs remain finite; zero/NaN/Inf rejected. | `transforms.quaternion_normalize` |
| 02 `hamilton_product_order` | X quarter-turn × Z quarter-turn equals matrix `Rx * Rz`; order and cross-product signs matter. | `quaternion_multiply` |
| 03 `rpy_fixed_axis_order` | Three nonzero RPY angles match `Rz(yaw) * Ry(pitch) * Rx(roll)`. | `quaternion_from_rpy` |
| 04 `axis_angle_normalizes_direction` | Oblique, negative and huge axes use unit direction; angle stays in radians. | `quaternion_from_axis_angle` |
| 05 `rotate_vector_handedness` | +Z quarter-turn maps `(2,-3,4)` → `(3,2,4)`; +X maps `(0,2,3)` → `(0,-3,2)`; zero stays zero. | `rotate_vector` cross products/signs |
| 06 `compose_rotates_child_translation` | General parent and child transforms equal independent `Tparent * Tchild`; rotate child offset before adding. | `compose_transforms` |
| 07 `matrix_column_major_layout` | General RPY and translation match all 16 matrix values; translation indices 12–14, last row 0/0/0/1. | `matrix_column_major` |
| 08 `urdf_defaults_namespace_direct_children` | Namespaced URDF; hidden transmission links/joints ignored; absent axis xyz defaults +X. | Description direct-child lookups, default axis |
| 09 `origin_xyz_is_parent_frame` | Nonzero mounted xyz with nonzero RPY: xyz stays `(1,2,3)`, never pre-rotated by its own RPY. | `_parse_origin`, `_parse_joint` **[practice 20]** |
| 10 `fixed_joint_ignores_position` | Fixed joint keeps origin at q=12.5; zero fixed axis allowed. | `Joint.joint_transform`, fixed dispatch |
| 11 `revolute_origin_then_motion` | Mounted oblique revolute: `Torigin * Raxis(q)`, including q outside limits. | `Joint.joint_transform` order; FK never clamps |
| 12 `continuous_negative_unwrapped_angle` | Negative angle −7.2 and negative axis; continuous limits ignored even if malformed. | Continuous type/axis handling |
| 13 `prismatic_preserves_axis_length` | Axis `(2,-3,4)`, q=.7 → xyz `(1.4,-2.1,2.8)` and identity rotation; negative q too. | Prismatic branch, axis scale **[practice 18]** |
| 14 `prismatic_motion_rotated_by_origin` | +Z quarter-turn mount, axis `(2,0,0)`, q=.7: xyz `(1,3.4,3)`. | Prismatic dispatch and `Torigin * Tmotion` **[18/20]** |
| 15 `urdf_rejects_invalid_topology` | Reject unknown/self/duplicate parents, duplicate links/joints, cycles, multiple roots and disconnection. | `_validate_tree`, name sets |
| 16 `visual_origin_never_changes_fk` | First visual parsed; later visuals ignored; visual xyz/RPY never moves link FK. | `_parse_links`, visual vs joint origin |
| 17 `jointstate_replacement_names_and_stamp` | Name-order lookup, last duplicate wins, unknown/fixed names ignored; missing joint resets zero, stamp preserved. | `accept_joint_states`, `joint_configuration` |
| 18 `tf_quaternion_xyzw_fields` | General oblique rotation emitted as named x/y/z/w fields with correct parent, child, stamp and translation. | `/tf` serialization **[practice 22]** |
| 19 `world_chain_rotates_offsets` | Rotating parent turns downstream fixed offset; root included; world matrix is full product along chain. | World stack/compose; local `/tf` upstream |
| 20 `world_branches_unordered_root` | Unsorted edges; both branches isolated; root from topology, every frame gets source stamp. | `_read_tree`, `_compose_tree` stack |
| 21 `global_pose_before_and_after_tf` | Pose before tree applies later; normalize pose quaternion; pose-only update moves tree and keeps stamp. | `accept_global_pose`, cached tree recomposition |
| 22 `description_rejection_is_atomic` | Bad reload preserves robot and q; correct rejected/loaded versions; later valid robot removes old links. | `_process_description`, `_replace_robot`, world cache |
| 23 `parameter_versions_missing_null` | Missing is false/null/version0; explicit null succeeds; every set increments only its name, even identical values. | `ParameterServer.get_param/set_param` |
| 24 `urdf_numeric_axis_limits_validation` | Reject malformed/nonfinite numbers, zero movable axis, missing required limit, unsupported type; absent bounds default zero without FK clamp. | Number/vector grammar, `_parse_joint` |
| 25 `invalid_messages_preserve_last_snapshot` | Bad joint arrays/types/stamps, duplicate-child/zero-quaternion TF, bad global pose cannot overwrite good snapshots. | Validate completely before assigning state |

Useful signatures: **13+14 fail** → prismatic type branch/translation. **09 and mounted-joint tests fail** → origin translation/order. **17+18+19 fail while 01–07 pass** → `/tf` field serialization. A failure in 01–07 can cascade into many FK tests; fix math first. These patterns are guidance, not proof when multiple bugs coexist.

The visualizer sends the chosen URDF, named joint positions and global pose to the running project and displays its robot pose. Use **Show pose** to update the view, drag to orbit, scroll to zoom, or choose **Fit view**. Cyan lines connect the robot links, and red/green/blue axes show each frame orientation. The page contains input controls and the resulting robot view. Invalid input or an unavailable runtime produces a general message.

All tests focus on the three graded nodes. The optional servo and FSM run only under `make demo`; their behavior is outside the grading contract. Passing these tests does not prove gateway routing, timing/rate, concurrency, long-message framing or every autograder case.
