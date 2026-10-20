# Fetch robot description: source and license

`fetch.urdf` is the Fetch Robotics Fetch robot description, redistributed
**unmodified** for this course.

- Source: https://github.com/fetchrobotics/fetch_ros at commit
  `d22b98f6c21577895f4d2e3a8bd54e329083732b`, file
  `fetch_description/robots/fetch.urdf`.
- sha256: `42c01b4faf7cbffdc92d2b1ecd655d02f2f33aa4df9fe4fc310a5661b0628b83` (22206 bytes).
- Copyright Fetch Robotics, Inc.
- License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0
  International (CC BY-NC-SA 4.0), as declared in
  `fetch_description/package.xml`: https://creativecommons.org/licenses/by-nc-sa/4.0/

Non-commercial use only. If you redistribute this file, or a modified version
of it, keep this attribution and the same license. Its `package://` mesh
references are not included. Forward kinematics never needs them; a viewer can
draw placeholder shapes for meshes.

Use it to try your runtime on a real robot. For example, set
`robot_description` to this file's contents with `/param_server/set_param`.
