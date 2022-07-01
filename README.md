# Bevy XPBD

This is a physics engine for bevy that is based on Extended Position Based Dynamics (XPBD).

This is an experimental work-in-progress that I am working on because I'm interested in learning about physics engines. You probably shouldn't use this for anything serious.

## Current features

- 2D and 3D support (3D should mostly work, but it's still a bit rough)
- Dynamic and static rigid bodies
- Position and rotation integration and solving
- Linear and angular velocity integration and solving
- Collision detection via [parry](https://parry.rs)
- Basic joints
  - Revolute joint (or hinge joint), optional angle limits
  - Spherical joint, optional swing and twist angle limits
  - Prismatic joint, one free translational axis with optional limits
  - Fixed joint
- Joint damping
- Gravity
- External forces
- Restitution
- Friction
- Substepping

## Future features

- Linear and angular velocity damping
- Locking translational and rotational axes without joints
- Collision groups
- Publicly accessable contact data
- Performance optimization (proper broadphase, sleeping, maybe simulation on the GPU...)
- Maybe basic soft bodies
- Maybe (unlikely) fluid simulation

## Other future work

- Documentation
- More examples
- Publish crate

## Inspirations

- [Johan Helsing's amazing but incomplete tutorial](https://johanhelsing.studio/posts/bevy_xpbd); I probably wouldn't have started without it
- The original papers
  - [XPBD: Position-Based Simulation of Compliant Constrained Dynamics](http://mmacklin.com/xpbd.pdf)
  - [Detailed Rigid Body Simulation with Extended Position Based Dynamics](https://matthias-research.github.io/pages/publications/PBDBodies.pdf) (I have used this one way more)
