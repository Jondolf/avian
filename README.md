# Bevy XPBD

This is an implementation of Extended Position Based Dynamics for bevy.

This is an experimental work-in-progress that I am working on because I'm interested in learning about physics engines. You most likely shouldn't use this for anything serious.

Currently this only works in 2D, but it should be relatively straightforward to add 3D support.

## Current features

- Position and linear velocity integration
- Gravity
- Collision detection via [parry](https://parry.rs)
- Position and linear velocity solving after dynamic-dynamic and dynamic-static contacts
- Restitution
- Substepping

## Future features

- Friction and drag
- Rotation and angular velocity
- External forces and maybe impulses
- 3D support
- Joints
- Maybe utils for rope and cloth simulation
- Maybe basic soft bodies
- Maybe (unlikely) fluid simulation

## Inspirations

- Heavily inspired by [Johan Helsing's tutorial](https://johanhelsing.studio/posts/bevy_xpbd)
- The original papers
  - [http://mmacklin.com/xpbd.pdf](http://mmacklin.com/xpbd.pdf)
  - [https://matthias-research.github.io/pages/publications/PBDBodies.pdf](https://matthias-research.github.io/pages/publications/PBDBodies.pdf)
