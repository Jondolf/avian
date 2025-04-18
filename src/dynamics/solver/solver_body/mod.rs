//! Efficient rigid body definitions used by the performance-critical solver.
//!
//! This helps improve memory locality and makes random access faster for the constraint solver.
//!
//! This includes the following types:
//!
//! - [`SolverBody`]: The body state used by the solver.
//! - [`SolverBodyInertia`]: The inertial properties of a body used by the solver.
//! - [`SolverBodyIndex`]: A component that stores the index of a [`SolverBody`] in the [`SolverBodies`] resource.
//! - [`SolverBodies`]: A resource that stores [`SolverBody`]s for awake and active rigid bodies.

mod plugin;

pub use plugin::SolverBodyPlugin;

use bevy::{
    prelude::*,
    tasks::{ParallelSliceMut, TaskPool},
};

use super::{Rotation, Vector};
use crate::{math::Scalar, prelude::LockedAxes, Tensor};

// The `SolverBody` layout is inspired by `b2BodyState` in Box2D v3.

/// Optimized rigid body state that the solver operates on,
/// designed to improve memory locality and performance.
///
/// Only awake dynamic bodies and kinematic bodies have an associated solver body
/// in the [`SolverBodies`] resource. Static bodies and sleeping dynamic bodies do not move,
/// so they instead use a "dummy state" with [`SolverBody::default()`].
///
/// # Representation
///
/// The solver doesn't have access to the position or rotation of static or sleeping bodies,
/// which is a problem when computing constraint anchors. To work around this, we have two options:
///
/// - **Option 1**: Use delta positions and rotations. This requires preparing
///   base anchors and other necessary positional data in world space,
///   and computing the updated anchors during substeps.
/// - **Option 2**: Use full positions and rotations. This requires storing
///   anchors in world space for static bodies and sleeping bodies,
///   and in local space for dynamic bodies.
///
/// Avian uses **Option 1**, because:
///
/// - Using delta positions reduces round-off error when bodies are far from the origin.
/// - Mixing world space and local space values depending on the body type would be
///   quite confusing and error-prone, and would possibly require more branching.
///
/// In addition to the delta position and rotation, we also store the linear and angular velocities
/// and some (currently unused) flags. This all fits in 32 bytes in 2D or 56 bytes in 3D with the `f32` feature.
///
/// The 2D data layout has been designed to support fast conversion to and from
/// wide SIMD types via scatter/gather operations in the future when SIMD optimizations
/// are implemented.
// TODO: Is there a better layout for 3D?
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
    // TODO: Flags indicating whether the body is kinematic and whether gyroscopic torque is enabled.
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

// TODO: Right now, the inertia tensor still stores 9 floats,
//       but it should store only 6 floats (symmetric matrix).
/*
Box2D v3 stores mass and angular inertia in constraint data.
For 2D, this is just 2 floats or 8 bytes for each body in each constraint.

However, we also support 3D and locking translational axes, so our worst case
would be *9* floats for each body, 3 for the effective mass vector
and 6 for the symmetric 3x3 inertia tensor. Storing 36 bytes
for each body in each constraint would be quite wasteful.

Instead, we store a separate `SolverBodyInertia` struct for each `SolverBody`.
The struct is optimized for memory locality and size.

In 2D, we store the effective inertial properties directly:

- Effective inverse mass (8 bytes)
- Effective inverse angular inertia (4 bytes)
- Flags (4 bytes)

for a total of 16 bytes.

In 3D, we instead compute the effective versions on the fly:

- Inverse mass (4 bytes)
- Inverse angular inertia (24 bytes, symmetric matrix with 6 floats)
- Flags (4 bytes)

for a total of 32 bytes.

The API abstracts over this difference in representation to reduce complexity.
*/

/// The inertial properties of a [`SolverBody`].
///
/// This includes the effective inverse mass and angular inertia,
/// and flags indicating whether the body is static or has locked axes.
///
/// 16 bytes in 2D and 32 bytes in 3D with the `f32` feature.
#[derive(Clone, Debug)]
pub struct SolverBodyInertia {
    /// The effective inverse mass of the body,
    /// taking into account any locked axes.
    ///
    /// 8 bytes with the `f32` feature.
    #[cfg(feature = "2d")]
    effective_inv_mass: Vector,

    /// The inverse mass of the body.
    ///
    /// 4 bytes with the `f32` feature.
    #[cfg(feature = "3d")]
    inv_mass: Scalar,

    /// The effective inverse angular inertia of the body,
    /// taking into account any locked axes.
    ///
    /// 4 bytes with the `f32` feature.
    #[cfg(feature = "2d")]
    effective_inv_inertia: Tensor,

    /// The world-space inverse angular inertia of the body
    /// before the substepping loop of the solver.
    ///
    /// Rotate by `delta_rotation` to get the current angular inertia
    /// during the substepping loop.
    ///
    /// 24 bytes with the `f32` feature.
    #[cfg(feature = "3d")]
    inv_inertia: Tensor,

    // TODO: We could also store the `Dominance` of the body here if we wanted to.
    // TODO: Technically we don't even need these flags at the moment.
    /// Flags indicating the inertial properties of the body,
    /// like locked axes and whether the body is static.
    ///
    /// 4 bytes.
    flags: InertiaFlags,
}

impl Default for SolverBodyInertia {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            effective_inv_mass: Vector::ZERO,
            #[cfg(feature = "3d")]
            inv_mass: 0.0,
            #[cfg(feature = "2d")]
            effective_inv_inertia: 0.0,
            #[cfg(feature = "3d")]
            inv_inertia: Tensor::ZERO,
            flags: InertiaFlags::STATIC,
        }
    }
}

/// Flags indicating the inertial properties of a body.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InertiaFlags(u32);

bitflags::bitflags! {
    impl InertiaFlags: u32 {
        /// Set if translation along the `X` axis is locked.
        const TRANSLATION_X_LOCKED = 0b100_000;
        /// Set if translation along the `Y` axis is locked.
        const TRANSLATION_Y_LOCKED = 0b010_000;
        /// Set if translation along the `Z` axis is locked.
        const TRANSLATION_Z_LOCKED = 0b001_000;
        /// Set if rotation around the `X` axis is locked.
        const ROTATION_X_LOCKED = 0b000_100;
        /// Set if rotation around the `Y` axis is locked.
        const ROTATION_Y_LOCKED = 0b000_010;
        /// Set if rotation around the `Z` axis is locked.
        const ROTATION_Z_LOCKED = 0b000_001;
        /// Set if the body has infinite mass.
        const INFINITE_MASS = 1 << 6;
        /// Set if the body has infinite inertia.
        const INFINITE_ANGULAR_INERTIA = 1 << 7;
        /// Set if all translational axes are locked.
        const TRANSLATION_LOCKED = Self::TRANSLATION_X_LOCKED.bits() | Self::TRANSLATION_Y_LOCKED.bits() | Self::TRANSLATION_Z_LOCKED.bits();
        /// Set if all rotational axes are locked.
        const ROTATION_LOCKED = Self::ROTATION_X_LOCKED.bits() | Self::ROTATION_Y_LOCKED.bits() | Self::ROTATION_Z_LOCKED.bits();
        /// Set if all translational and rotational axes are locked.
        const ALL_LOCKED = Self::TRANSLATION_LOCKED.bits() | Self::ROTATION_LOCKED.bits();
        /// Set if the body is static.
        const STATIC = Self::INFINITE_MASS.bits() | Self::INFINITE_ANGULAR_INERTIA.bits();
    }
}

impl SolverBodyInertia {
    /// Creates a new [`SolverBodyInertia`] with the given mass, angular inertia,
    /// and locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn new(inv_mass: Scalar, inv_inertia: Tensor, locked_axes: LockedAxes) -> Self {
        let mut effective_inv_mass = Vector::splat(inv_mass);
        let mut effective_inv_inertia = inv_inertia;
        let mut flags = InertiaFlags(locked_axes.to_bits() as u32);

        if inv_mass == 0.0 {
            flags |= InertiaFlags::INFINITE_MASS;
        }
        if inv_inertia == 0.0 {
            flags |= InertiaFlags::INFINITE_ANGULAR_INERTIA;
        }

        if locked_axes.is_translation_x_locked() {
            effective_inv_mass.x = 0.0;
        }
        if locked_axes.is_translation_y_locked() {
            effective_inv_mass.y = 0.0;
        }
        if locked_axes.is_rotation_locked() {
            effective_inv_inertia = 0.0;
        }

        Self {
            effective_inv_mass,
            effective_inv_inertia,
            flags: InertiaFlags(flags.0),
        }
    }

    /// Creates a new [`SolverBodyInertia`] with the given mass, angular inertia,
    /// and locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn new(inv_mass: Scalar, inv_inertia: Tensor, locked_axes: LockedAxes) -> Self {
        let mut flags = InertiaFlags(locked_axes.to_bits() as u32);

        if inv_mass == 0.0 {
            flags |= InertiaFlags::INFINITE_MASS;
        }
        if inv_inertia == Tensor::ZERO {
            flags |= InertiaFlags::INFINITE_ANGULAR_INERTIA;
        }

        Self {
            inv_mass,
            inv_inertia,
            flags: InertiaFlags(flags.0),
        }
    }

    /// Returns the effective inverse mass of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn effective_inv_mass(&self) -> Vector {
        self.effective_inv_mass
    }

    /// Returns the effective inverse mass of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn effective_inv_mass(&self) -> Vector {
        let mut inv_mass = Vector::splat(self.inv_mass);

        if self.flags.contains(InertiaFlags::TRANSLATION_X_LOCKED) {
            inv_mass.x = 0.0;
        }
        if self.flags.contains(InertiaFlags::TRANSLATION_Y_LOCKED) {
            inv_mass.y = 0.0;
        }
        if self.flags.contains(InertiaFlags::TRANSLATION_Z_LOCKED) {
            inv_mass.z = 0.0;
        }

        inv_mass
    }

    /// Returns the effective inverse angular inertia of the body,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "2d")]
    pub fn effective_inv_angular_inertia(&self, _delta_rotation: &Rotation) -> Tensor {
        self.effective_inv_inertia
    }

    /// Returns the effective inverse angular inertia of the body in world space,
    /// taking into account any locked axes.
    #[inline]
    #[cfg(feature = "3d")]
    pub fn effective_inv_angular_inertia(&self, delta_rotation: &Rotation) -> Tensor {
        use crate::prelude::ComputedAngularInertia;

        let mut world_inv_inertia = ComputedAngularInertia::from_inverse_tensor(self.inv_inertia)
            .rotated(delta_rotation.0)
            .inverse();

        if self.flags.contains(InertiaFlags::ROTATION_X_LOCKED) {
            world_inv_inertia.x_axis = Vector::ZERO;
        }
        if self.flags.contains(InertiaFlags::ROTATION_Y_LOCKED) {
            world_inv_inertia.y_axis = Vector::ZERO;
        }
        if self.flags.contains(InertiaFlags::ROTATION_Z_LOCKED) {
            world_inv_inertia.z_axis = Vector::ZERO;
        }

        world_inv_inertia
    }

    /// Returns the [`InertiaFlags`] of the body.
    #[inline]
    pub fn flags(&self) -> InertiaFlags {
        self.flags
    }
}

// TODO: Should this be an immutable component? We could also only expose a read-only API.
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

/// A resource that stores [solver bodies](SolverBody) for awake and active rigid bodies.
///
/// Each body stores an index into this vector in the [`SolverBodyIndex`] component.
#[derive(Resource, Default)]
pub struct SolverBodies {
    // TODO: Use `UniqueEntityVec`.
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
    inertial_properties: Vec<SolverBodyInertia>,
}

impl SolverBodies {
    /// Creates a new empty collection of solver bodies.
    #[inline]
    pub const fn new() -> Self {
        Self {
            entities: Vec::new(),
            bodies: Vec::new(),
            inertial_properties: Vec::new(),
        }
    }

    /// Gets the [`Entity`] associated with the given solver body index.
    #[inline]
    pub fn get_entity(&self, index: SolverBodyIndex) -> Option<Entity> {
        self.entities.get(index.0).copied()
    }

    /// Returns an iterator over the solver bodies.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &SolverBody> {
        self.bodies.iter()
    }

    /// Returns an iterator over mutable references to the solver bodies.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut SolverBody> {
        self.bodies.iter_mut()
    }

    /// Splits the slice into a maximum of `max_tasks` chunks, and maps the chunks in parallel
    /// across the provided `task_pool`. One task is spawned in the task pool for every chunk.
    ///
    /// See [`ParallelSliceMut::par_splat_map_mut`] for more information.
    #[inline]
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
    #[inline]
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Returns `true` if there are no solver bodies.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Adds a new solver body for the given entity.
    #[inline]
    pub fn push(
        &mut self,
        entity: Entity,
        body: SolverBody,
        inertial_properties: SolverBodyInertia,
    ) -> SolverBodyIndex {
        let index = SolverBodyIndex(self.bodies.len());
        self.entities.push(entity);
        self.bodies.push(body);
        self.inertial_properties.push(inertial_properties);
        index
    }

    /// Removes a solver body and returns it along with its entity and inertial properties.
    ///
    /// The removed body is replaced by the last body in the vector.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    #[inline]
    pub fn swap_remove(
        &mut self,
        index: SolverBodyIndex,
    ) -> (Entity, SolverBody, SolverBodyInertia) {
        let entity = self.entities.swap_remove(index.0);
        let body = self.bodies.swap_remove(index.0);
        let inertial_properties = self.inertial_properties.swap_remove(index.0);
        (entity, body, inertial_properties)
    }

    /// Clears all solver bodies.
    #[inline]
    pub fn clear(&mut self) {
        self.entities.clear();
        self.bodies.clear();
    }

    /// Returns `true` if the collection contains the given entity.
    #[inline]
    pub fn contains_entity(&self, entity: Entity) -> bool {
        self.entities.contains(&entity)
    }

    /// Returns `true` if the collection contains the given solver body index.
    #[inline]
    pub fn contains_index(&self, index: SolverBodyIndex) -> bool {
        index.0 < self.bodies.len()
    }

    /// Returns a reference to the solver body with the given index.
    #[inline]
    pub fn get(&self, index: SolverBodyIndex) -> Option<&SolverBody> {
        self.bodies.get(index.0)
    }

    /// Returns a mutable reference to the solver body with the given index.
    #[inline]
    pub fn get_mut(&mut self, index: SolverBodyIndex) -> Option<&mut SolverBody> {
        self.bodies.get_mut(index.0)
    }

    /// Returns a mutable reference to the solver body with the given index without doing bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the index is in bounds.
    #[inline]
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
    ) -> (
        Option<(&mut SolverBody, &SolverBodyInertia)>,
        Option<(&mut SolverBody, &SolverBodyInertia)>,
    ) {
        let [min, max] = if a.0 < b.0 { [a.0, b.0] } else { [b.0, a.0] };
        if max == SolverBodyIndex::INVALID.0 {
            let body = self.bodies.get_unchecked_mut(min);
            if a.0 == max {
                (None, Some((body, &self.inertial_properties[min])))
            } else {
                (Some((body, &self.inertial_properties[min])), None)
            }
        } else {
            let first = &mut *(self.bodies.get_unchecked_mut(min) as *mut _);
            let second = &mut *(self.bodies.get_unchecked_mut(max) as *mut _);
            if a.0 == max {
                (
                    Some((second, &self.inertial_properties[max])),
                    Some((first, &self.inertial_properties[min])),
                )
            } else {
                (
                    Some((first, &self.inertial_properties[min])),
                    Some((second, &self.inertial_properties[max])),
                )
            }
        }
    }

    /// Returns a reference to the inertial properties of the body
    /// with the given index.
    #[inline]
    pub fn get_inertial_properties(&self, index: SolverBodyIndex) -> Option<&SolverBodyInertia> {
        self.inertial_properties.get(index.0)
    }

    /// Returns a mutable reference to the inertial properties of the body
    /// with the given index.
    #[inline]
    pub fn get_inertial_properties_mut(
        &mut self,
        index: SolverBodyIndex,
    ) -> Option<&mut SolverBodyInertia> {
        self.inertial_properties.get_mut(index.0)
    }

    /// Returns a mutable reference to the inertial properties of the body
    /// with the given index without doing bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the index is in bounds.
    #[inline]
    pub unsafe fn get_inertial_properties_unchecked_mut(
        &mut self,
        index: SolverBodyIndex,
    ) -> &mut SolverBodyInertia {
        self.inertial_properties.get_unchecked_mut(index.0)
    }
}
