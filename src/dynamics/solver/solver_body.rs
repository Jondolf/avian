//! Efficient rigid body definitions used by the performance-critical solver.
//!
//! This helps improve memory locality and makes random access faster for the constraint solver.

use bevy::{
    prelude::*,
    tasks::{ParallelSliceMut, TaskPool},
};

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
#[derive(Resource, Default)]
pub struct SolverBodies {
    // TODO: Use `UniqueEntityVec` in Bevy 0.16.
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
}

impl SolverBodies {
    /// Creates a new empty collection of solver bodies.
    pub const fn new() -> Self {
        Self {
            entities: Vec::new(),
            bodies: Vec::new(),
        }
    }

    /// Gets the [`Entity`] associated with the given solver body index.
    pub fn get_entity(&self, index: SolverBodyIndex) -> Option<Entity> {
        self.entities.get(index.0).copied()
    }

    /// Returns an iterator over the solver bodies.
    pub fn iter(&self) -> impl Iterator<Item = &SolverBody> {
        self.bodies.iter()
    }

    /// Returns an iterator over mutable references to the solver bodies.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut SolverBody> {
        self.bodies.iter_mut()
    }

    /// Splits the slice into a maximum of `max_tasks` chunks, and maps the chunks in parallel
    /// across the provided `task_pool`. One task is spawned in the task pool for every chunk.
    ///
    /// See [`ParallelSliceMut::par_splat_map_mut`] for more information.
    pub fn par_splat_map_mut<F, R>(
        &mut self,
        task_pool: &TaskPool,
        max_tasks: Option<usize>,
        f: F,
    ) -> Vec<R>
    where
        F: Fn(usize, &mut [SolverBody]) -> R + Send + Sync,
        R: Send + 'static,
    {
        self.bodies.par_splat_map_mut(task_pool, max_tasks, f)
    }

    /// Returns the number of solver bodies.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Returns `true` if there are no solver bodies.
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Adds a new solver body for the given entity.
    pub fn push(&mut self, entity: Entity, body: SolverBody) -> SolverBodyIndex {
        let index = SolverBodyIndex(self.bodies.len());
        self.entities.push(entity);
        self.bodies.push(body);
        index
    }

    /// Removes a solver body and returns it.
    ///
    /// The removed body is replaced by the last body in the vector.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    pub fn swap_remove(&mut self, index: SolverBodyIndex) -> (Entity, SolverBody) {
        let entity = self.entities.swap_remove(index.0);
        let body = self.bodies.swap_remove(index.0);
        (entity, body)
    }

    /// Clears all solver bodies.
    pub fn clear(&mut self) {
        self.entities.clear();
        self.bodies.clear();
    }

    /// Returns `true` if the collection contains the given entity.
    pub fn contains_entity(&self, entity: Entity) -> bool {
        self.entities.contains(&entity)
    }

    /// Returns `true` if the collection contains the given solver body index.
    pub fn contains_index(&self, index: SolverBodyIndex) -> bool {
        index.0 < self.bodies.len()
    }

    /// Returns a reference to the solver body with the given index.
    pub fn get(&self, index: SolverBodyIndex) -> Option<&SolverBody> {
        self.bodies.get(index.0)
    }

    /// Returns a mutable reference to the solver body with the given index.
    pub fn get_mut(&mut self, index: SolverBodyIndex) -> Option<&mut SolverBody> {
        self.bodies.get_mut(index.0)
    }

    /// Returns a mutable reference to the solver body with the given index without doing bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the index is in bounds.
    pub unsafe fn get_unchecked_mut(&mut self, index: usize) -> &mut SolverBody {
        self.bodies.get_unchecked_mut(index)
    }

    /// Returns mutable references to the two solver bodies with the given indices.
    ///
    /// If a given index is [`SolverBodyIndex::INVALID`], the corresponding body will be `None`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `a != b` if the indices are not [`SolverBodyIndex::INVALID`].
    #[inline]
    pub unsafe fn get_pair_unchecked_mut(
        &mut self,
        a: SolverBodyIndex,
        b: SolverBodyIndex,
    ) -> (Option<&mut SolverBody>, Option<&mut SolverBody>) {
        let [min, max] = if a.0 < b.0 { [a.0, b.0] } else { [b.0, a.0] };
        if max == SolverBodyIndex::INVALID.0 {
            let body = self.bodies.get_unchecked_mut(min);
            if a.0 == max {
                (None, Some(body))
            } else {
                (Some(body), None)
            }
        } else {
            let first = &mut *(self.bodies.get_unchecked_mut(min) as *mut _);
            let second = &mut *(self.bodies.get_unchecked_mut(max) as *mut _);
            if a.0 == max {
                (Some(second), Some(first))
            } else {
                (Some(first), Some(second))
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
