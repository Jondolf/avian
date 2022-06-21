# Bevy XPBD

This is a physics engine for bevy that is based on Extended Position Based Dynamics (XPBD).

This is an experimental work-in-progress that I am working on because I'm interested in learning about physics engines. You probably shouldn't use this for anything serious.

## Current features

- 2D and 3D support (3D should mostly work, but it's still a bit rough)
- Dynamic and static rigid bodies
- Position and rotation integration and solving
- Linear and angular velocity integration and solving
- Collision detection via [parry](https://parry.rs)
- Gravity
- Restitution
- Friction
- Substepping

## Future features

- Linear and angular velocity damping
- External forces and maybe impulses
- Joints
- Maybe utils for rope and cloth simulation
- Maybe basic soft bodies
- Maybe (unlikely) fluid simulation

## Inspirations

- [Johan Helsing's amazing but incomplete tutorial](https://johanhelsing.studio/posts/bevy_xpbd); I probably wouldn't have started without it
- The original papers
  - [XPBD: Position-Based Simulation of Compliant Constrained Dynamics](http://mmacklin.com/xpbd.pdf)
  - [Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf) (I have used this one way more)
