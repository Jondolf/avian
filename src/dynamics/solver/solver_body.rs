//! Efficient rigid body definitions used by the performance-critical solver.
//!
//! This helps improve memory locality and makes random access faster for the constraint solver.

use bevy::prelude::*;

use super::{Rotation, Vector};
#[cfg(feature = "2d")]
use crate::math::Scalar;

// In the future, this will also be used for fast conversion to/from SIMD types via scatter/gather.
// TODO: Is there a better layout for 3D?
/// An optimized representation of rigid body data used by the solver,
/// designed to improve memory locality and performance.
///
/// Only awake dynamic bodies and kinematic bodies have an associated solver body.
/// Static bodies and sleeping dynamic bodies do not move, and are not included in the solver.
///
/// 32 bytes in 2D and 56 bytes in 3D with the `f32` feature.
#[derive(Clone, Debug, Default)]
pub struct SolverBody {
    /// The linear velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    pub linear_velocity: Vector,
    /// The angular velocity of the body.
    ///
    /// 4 bytes in 2D and 8 bytes in 3D with the `f32` feature.
    #[cfg(feature = "2d")]
    pub angular_velocity: Scalar,
    /// The angular velocity of the body.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    #[cfg(feature = "3d")]
    pub angular_velocity: Vector,
    /// The change in position of the body.
    ///
    /// Stored as a delta to avoid round-off error when far from the origin.
    ///
    /// 8 bytes in 2D and 12 bytes in 3D with the `f32` feature.
    pub delta_position: Vector,
    /// The change in rotation of the body.
    ///
    /// Stored as a delta because the rotation of static bodies cannot be accessed
    /// in the solver, but they have a known delta rotation of zero.
    ///
    /// 8 bytes in 2D and 16 bytes in 3D with the `f32` feature.
    pub delta_rotation: Rotation,
    /// Flags for the body.
    ///
    /// 4 bytes.
    pub flags: u32,
}

impl SolverBody {
    /// Computes the velocity at the given `point` relative to the center of the body.
    pub fn velocity_at_point(&self, point: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.linear_velocity + self.angular_velocity * point.perp()
        }
        #[cfg(feature = "3d")]
        {
            self.linear_velocity + self.angular_velocity.cross(point)
        }
    }
}

/// A resource that stores [solver bodies](SolverBody) for awake and active rigid bodies.
///
/// Each body stores an index into this vector in the [`SolverBodyIndex`] component.
#[derive(Resource, Default, Deref, DerefMut)]
pub struct SolverBodies(pub Vec<SolverBody>);

impl SolverBodies {
    /// Returns mutable references to the two solver bodies with the given indices.
    ///
    /// If a given index is [`SolverBodyIndex::INVALID`], the corresponding body will be `None`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `a != b` if the indices are not [`SolverBodyIndex::INVALID`].
    pub unsafe fn get_pair_unchecked(
        &mut self,
        a: SolverBodyIndex,
        b: SolverBodyIndex,
    ) -> (Option<&mut SolverBody>, Option<&mut SolverBody>) {
        let min = a.min(b.0);
        let max = a.max(b.0);

        if max == SolverBodyIndex::INVALID.0 {
            let body = self.get_unchecked_mut(min);
            if a.0 == max {
                (None, Some(body))
            } else {
                (Some(body), None)
            }
        } else {
            let (first, second) = self.split_at_mut(max);
            let first = first.get_unchecked_mut(min);
            let second = Some(second.get_unchecked_mut(0));
            if a.0 == max {
                (second, Some(first))
            } else {
                (Some(first), second)
            }
        }
    }
}

/// A component that stores the index of a [`SolverBody`] in the [`SolverBodies`] resource.
#[derive(Component, Clone, Copy, Debug, Deref, PartialEq, Eq, Reflect)]
#[reflect(Component, Debug, PartialEq)]
pub struct SolverBodyIndex(pub usize);

impl SolverBodyIndex {
    /// An invalid index that can be used to indicate that the body is not an awake dynamic body.
    pub const INVALID: Self = Self(usize::MAX);

    /// Returns `true` if the index represents a valid awake dynamic body.
    pub fn is_valid(&self) -> bool {
        self.0 != usize::MAX
    }
}
