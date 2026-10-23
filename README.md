# Project 3: Forward Kinematics and Robot Choreography

**ROB 511 · Mobile Manipulation**

A robot description, joint feedback, and a finite state machine bring **Luma**, a
custom inspection arm, to life. This project computes every link's pose in the
world and displays the resulting choreography in a viewer built for the project.

## Choreography video

<!-- Replace the line above with:
**Screen recording of the live dance:** [Watch Luma perform its choreography](YOUR_VIDEO_URL)
Record the live viewer while make demo is running.
-->

The preview below was exported from transforms captured from the running robot
software. The space above is reserved for a screen recording of the live demo.

<video controls playsinline preload="metadata" poster="portfolio/poster.png" style="width: 100%; max-width: 960px;">
  <source src="portfolio/demo.webm" type="video/webm">
  <a href="portfolio/demo.webm">Watch the recorded runtime preview.</a>
</video>

[Watch or download the 24-second preview](portfolio/demo.webm) ·
[Explore the interactive replay](https://ntsai04.github.io/ROB-511-Mobile-Manipulation-Project-3/viewer/?replay=../portfolio/demo_recording.json)

The replay supports dragging to orbit and scrolling to zoom. It uses captured
runtime messages, so it can be viewed without starting the robot software.

## The robot

Luma has **twelve links and six movable joints**: a rotating base, shoulder, elbow,
wrist roll, and two opposing sliding gripper fingers. Fixed joints attach details
such as the base ring, camera shell, and eyes. The robot is described in
[a custom URDF](examples/showcase.urdf) using boxes, cylinders, and spheres.

The different joint types make the motion easy to see: the arm rotates to reach
and wave, while the fingers translate to open and close the gripper.

## The choreography

The [choreography table](examples/choreography.json) defines eight states in a loop:

`wake → look_left → wave_high → wave_low → reach → grasp → present → bow → wake`

Luma wakes up, turns to look left, and waves by rolling its wrist between two
poses. It then reaches forward, closes its gripper, turns to present it, and bows
before starting again.

Each state specifies joint positions and a named transition. On entry, the finite
state machine publishes the target on `/joint_trajectory`. A proportional joint
servo moves toward it and publishes `/joint_states`. The FSM waits until every
targeted joint is within the state's tolerance, then holds the pose briefly before
following the next transition. The sequence therefore advances using joint
feedback; one loop takes approximately twenty seconds.

## How I drew it

I built a browser viewer using JavaScript and Canvas, with a small Python adapter
acting as an **external client** of the runtime. The adapter subscribes over the
same TCP/JSON protocol used by other clients and forwards messages to the browser
through Server-Sent Events.

The robot state publisher computes parent-to-child transforms on `/tf`. The world
state publisher composes the tree with the root's global pose and publishes every
link on `/xform_world`. The viewer uses those supplied world transforms to place
the links.

For each link, the viewer reads its first URDF visual and draws its primitive
geometry. It multiplies the link's world matrix by the visual's local origin:
`T_global_visual = T_global_link × T_link_visual`. The incoming matrices contain
16 numbers in **column-major order**, with translation at indices 12, 13, and 14.
A perspective camera projects the transformed geometry onto the canvas. Orbiting
the camera changes the view while the robot's world coordinates remain z-up.

[Viewer source](viewer/viewer.js) · [TCP-to-browser adapter](tools/viewer_server.py)

## Run and record the dance

From the project directory, build once, then run these commands in two terminals:

```sh
make build
make demo
```

```sh
make viewer
```

Open **http://127.0.0.1:8080/viewer/** and screen-record the robot moving through
the choreography. `make demo` loads the custom Luma robot and choreography shown
above. Paste the uploaded recording's link into the video section of this README.

See the [running and recording guide](docs/RUNNING.md) for Windows commands,
runtime services, and submission instructions, or the
[code walkthrough](docs/CODE_WALKTHROUGH.md) for the implementation.

## Credits

Luma's URDF and primitive geometry were created for this project. The repository
also includes the course kit's Fetch description:
**Fetch robot description © Fetch Robotics, Inc., CC BY-NC-SA 4.0**.
Its [license](robots/fetch/LICENSE.md) is included with the description.
