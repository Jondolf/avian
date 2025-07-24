//! Common components and bundles for rigid bodies.

pub mod forces;
pub mod mass_properties;

// Components
mod locked_axes;
mod physics_material;

pub use locked_axes::LockedAxes;
pub use physics_material::{
    CoefficientCombine, DefaultFriction, DefaultRestitution, Friction, Restitution,
};

#[cfg(feature = "2d")]
pub(crate) use forces::FloatZero;

use crate::{
    physics_transform::init_physics_transform,
    prelude::{forces::AccumulatedLocalAcceleration, *},
};
use bevy::{
    ecs::{component::HookContext, world::DeferredWorld},
    prelude::*,
};
use derive_more::From;

/// A non-deformable body used for the simulation of most physics objects.
///
/// # Rigid Body Types
///
/// A rigid body can be either dynamic, kinematic or static.
///
/// - **Dynamic bodies** are similar to real life objects and are affected by forces and contacts.
/// - **Kinematic bodies** can only be moved programmatically, which is useful for things like character controllers and moving platforms.
/// - **Static bodies** can not move, so they can be good for objects in the environment like the ground and walls.
///
/// # Creation
///
/// Creating a rigid body is as simple as adding the [`RigidBody`] component,
/// and an optional [`Collider`]:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body and specify its position.
///     commands.spawn((
///         DynamicBody,
///         Collider::capsule(0.5, 1.5),
///         Transform::from_xyz(0.0, 3.0, 0.0),
///     ));
/// }
/// ```
///
/// By default, dynamic rigid bodies will have mass properties computed based on the attached colliders
/// and their [`ColliderDensity`]. See the [Mass properties](#mass-properties) section for more information.
///
/// # Movement
///
/// A rigid body can be moved in three ways: by modifying its position directly,
/// by changing its velocity, or by applying forces or impulses.
///
/// To change the position of a rigid body, you can simply modify its `Transform`:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn move_bodies(mut query: Query<&mut Transform, With<RigidBody>>) {
///     for mut transform in &mut query {
///         transform.translation.x += 0.1;
///     }
/// }
/// ```
///
/// However, moving a dynamic body by changing its position directly is similar
/// to teleporting the body, which can result in unexpected behavior since the body can move
/// inside walls.
///
/// You can instead change the velocity of a dynamic or kinematic body with the [`LinearVelocity`]
/// and [`AngularVelocity`] components:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn accelerate_bodies(
///     mut query: Query<(&mut LinearVelocity, &mut AngularVelocity)>,
///     time: Res<Time>,
/// ) {
///     let delta_secs = time.delta_secs();
///     for (mut linear_velocity, mut angular_velocity) in &mut query {
///         linear_velocity.x += 2.0 * delta_secs;
#[cfg_attr(
    feature = "2d",
    doc = "        angular_velocity.0 += 0.5 * delta_secs;"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        angular_velocity.z += 0.5 * delta_secs;"
)]
///     }
/// }
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// For applying forces, impulses, and acceleration to dynamic bodies, see the [`forces`] module.
///
/// Avian does not have a built-in character controller, so if you need one,
/// you will need to implement it yourself or use a third party option.
/// You can take a look at the [`basic_dynamic_character`] and [`basic_kinematic_character`]
/// examples for a simple implementation.
///
/// [`basic_dynamic_character`]: https://github.com/Jondolf/avian/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/avian3d/examples/basic_dynamic_character.rs
/// [`basic_kinematic_character`]: https://github.com/Jondolf/avian/blob/42fb8b21c756a7f4dd91071597dc251245ddaa8f/crates/avian3d/examples/basic_kinematic_character.rs
///
/// # Mass Properties
///
/// Every dynamic rigid body has [mass], [angular inertia], and a [center of mass].
/// These mass properties determine how the rigid body responds to forces and torques.
///
/// - **Mass**: Represents resistance to linear acceleration. A higher mass requires more force for the same acceleration.
/// - **Angular Inertia**: Represents resistance to angular acceleration. A higher angular inertia requires more torque for the same angular acceleration.
/// - **Center of Mass**: The average position of mass in the body. Applying forces at this point produces no torque.
///
/// Static and kinematic rigid bodies have infinite mass and angular inertia,
/// and do not respond to forces or torques. Zero mass for a dynamic body is also
/// treated as a special case, and corresponds to infinite mass.
///
/// If no mass properties are set, they are computed automatically from attached colliders
/// based on their shape and density.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Note: `ColliderDensity` is optional, and defaults to `1.0` if not present.
/// commands.spawn((
///     DynamicBody,
///     Collider::capsule(0.5, 1.5),
///     ColliderDensity(2.0),
/// ));
/// # }
/// ```
///
/// If mass properties are set with the [`Mass`], [`AngularInertia`], and [`CenterOfMass`] components,
/// they override the values computed from colliders.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Override mass and the center of mass, but use the collider's angular inertia.
/// commands.spawn((
///     DynamicBody,
///     Collider::capsule(0.5, 1.5),
///     Mass(5.0),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
/// ));
/// # }
/// ```
///
/// If the rigid body has child colliders, their mass properties will be combined for
/// the total [`ComputedMass`], [`ComputedAngularInertia`], and [`ComputedCenterOfMass`].
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total mass: 10.0 + 5.0 = 15.0
#[cfg_attr(
    feature = "2d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5] + 5.0 * [0.0, 4.0]) / (10.0 + 5.0) = [0.0, 1.0]"
)]
#[cfg_attr(
    feature = "3d",
    doc = "// Total center of mass: (10.0 * [0.0, -0.5, 0.0] + 5.0 * [0.0, 4.0, 0.0]) / (10.0 + 5.0) = [0.0, 1.0, 0.0]"
)]
/// commands.spawn((
///     DynamicBody,
///     Collider::capsule(0.5, 1.5),
///     Mass(10.0),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
///     Transform::default(),
/// ))
/// .with_child((
#[cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
///     Mass(5.0),
///     Transform::from_xyz(0.0, 4.0, 0.0),
/// ));
/// # }
/// ```
///
/// To prevent child entities from contributing to the total mass properties, use the [`NoAutoMass`],
/// [`NoAutoAngularInertia`], and [`NoAutoCenterOfMass`] marker components.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Total mass: 10.0
#[cfg_attr(feature = "2d", doc = "// Total center of mass: [0.0, -0.5]")]
#[cfg_attr(feature = "3d", doc = "// Total center of mass: [0.0, -0.5, 0.0]")]
/// commands.spawn((
///     DynamicBody,
///     Collider::capsule(0.5, 1.5),
///     Mass(10.0),
#[cfg_attr(feature = "2d", doc = "    CenterOfMass::new(0.0, -0.5),")]
#[cfg_attr(feature = "3d", doc = "    CenterOfMass::new(0.0, -0.5, 0.0),")]
///     NoAutoMass,
///     NoAutoCenterOfMass,
///     Transform::default(),
/// ))
/// .with_child((
#[cfg_attr(feature = "2d", doc = "    Collider::circle(1.0),")]
#[cfg_attr(feature = "3d", doc = "    Collider::sphere(1.0),")]
///     Mass(5.0),
///     Transform::from_xyz(0.0, 4.0, 0.0),
/// ));
/// # }
/// ```
///
/// See the [`mass_properties`] module for more information.
///
/// [mass]: mass_properties::components::Mass
/// [angular inertia]: mass_properties::components::AngularInertia
/// [center of mass]: mass_properties::components::CenterOfMass
/// [mass properties]: mass_properties
///
/// # See More
///
/// - [Colliders](Collider)
/// - [Gravity] and [gravity scale](GravityScale)
/// - [Linear](LinearDamping) and [angular](AngularDamping) velocity damping
/// - [Friction] and [restitution](Restitution) (bounciness)
/// - [Lock translational and rotational axes](LockedAxes)
/// - [Dominance]
/// - [Continuous Collision Detection](dynamics::ccd)
///     - [Speculative collision](dynamics::ccd#speculative-collision)
///     - [Swept CCD](dynamics::ccd#swept-ccd)
/// - [`Transform` interpolation and extrapolation](PhysicsInterpolationPlugin)
/// - [Temporarily disabling a rigid body](RigidBodyDisabled)
/// - [Automatic deactivation with sleeping](Sleeping)
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(
    // TODO: Only dynamic and kinematic bodies need velocity,
    //       and only dynamic bodies need mass and angular inertia.
    Position::PLACEHOLDER,
    Rotation::PLACEHOLDER,
)]
#[component(on_add = RigidBody::on_add)]
pub struct RigidBody;

impl RigidBody {
    /// Checks if the rigid body is dynamic.
    #[deprecated(
        note = "The `is_dynamic` method is implemented through the `PartialReflect` trait, but it does not return whether the rigid body is dynamic. Check for the `DynamicBody` component instead, or if you really meant `PartialReflect::is_dynamic`, use it explicitly."
    )]
    #[doc(hidden)]
    pub fn is_dynamic(&self) -> bool {
        panic!(
            "The `is_dynamic` method is implemented through the `PartialReflect` trait, but it does not return whether the rigid body is dynamic. Check for the `DynamicBody` component instead, or if you really meant `PartialReflect::is_dynamic`, use it explicitly."
        );
    }

    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        // Initialize the global physics transform for the rigid body.
        init_physics_transform(&mut world, &ctx);
    }
}

/// A [`RigidBody`] with [mass properties](mass_properties) and [velocity] that is simulated and affected by forces.
#[derive(Component, Reflect, Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(
    RigidBody,
    LinearVelocity,
    AngularVelocity,
    ComputedMass,
    ComputedAngularInertia,
    ComputedCenterOfMass,
    // Required for local forces and acceleration.
    AccumulatedLocalAcceleration,
    // TODO: We can remove these pre-solve deltas once joints don't use XPBD.
    PreSolveDeltaPosition,
    PreSolveDeltaRotation,
)]
#[component(on_add = DynamicBody::on_add, on_remove = DynamicBody::on_remove)]
pub struct DynamicBody;

impl DynamicBody {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);
        let is_kinematic = entity_ref.contains::<KinematicBody>();
        let is_static = entity_ref.contains::<StaticBody>();

        // If the entity has a `KinematicBody` or `StaticBody`, replace it.
        if is_kinematic {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a dynamic body.
                    if entity_mut.contains::<DynamicBody>() {
                        entity_mut.remove::<KinematicBody>();
                    }
                }
            });
        }
        if is_static {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a dynamic body.
                    if entity_mut.contains::<DynamicBody>() {
                        entity_mut.remove::<StaticBody>();
                    }
                }
            });
        }
    }

    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);

        // If the entity has no rigid body type left, remove `RigidBody`.
        if !entity_ref.contains::<DynamicBody>()
            && !entity_ref.contains::<KinematicBody>()
            && !entity_ref.contains::<StaticBody>()
        {
            world.commands().queue(move |world: &mut World| {
                // Only remove the component if the entity has no rigid body type left.
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    if !entity_mut.contains::<DynamicBody>()
                        && !entity_mut.contains::<KinematicBody>()
                        && !entity_mut.contains::<StaticBody>()
                    {
                        entity_mut.remove::<RigidBody>();
                    }
                }
            });
        }
    }
}

/// A [`RigidBody`] with [velocity] that is moved programmatically and does not respond to forces or collisions.
#[derive(Component, Reflect, Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(RigidBody, LinearVelocity, AngularVelocity)]
#[component(on_add = KinematicBody::on_add, on_remove = KinematicBody::on_remove)]
pub struct KinematicBody;

impl KinematicBody {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);
        let is_dynamic = entity_ref.contains::<DynamicBody>();
        let is_static = entity_ref.contains::<StaticBody>();

        // If the entity has a `DynamicBody` or `StaticBody`, replace it.
        if is_dynamic {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a kinematic body.
                    if entity_mut.contains::<KinematicBody>() {
                        entity_mut.remove::<DynamicBody>();
                    }
                }
            });
        }
        if is_static {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a kinematic body.
                    if entity_mut.contains::<KinematicBody>() {
                        entity_mut.remove::<StaticBody>();
                    }
                }
            });
        }
    }

    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);

        // If the entity has no rigid body type left, remove `RigidBody`.
        if !entity_ref.contains::<DynamicBody>()
            && !entity_ref.contains::<KinematicBody>()
            && !entity_ref.contains::<StaticBody>()
        {
            world.commands().queue(move |world: &mut World| {
                // Only remove the component if the entity has no rigid body type left.
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    if !entity_mut.contains::<DynamicBody>()
                        && !entity_mut.contains::<KinematicBody>()
                        && !entity_mut.contains::<StaticBody>()
                    {
                        entity_mut.remove::<RigidBody>();
                    }
                }
            });
        }
    }
}

/// A [`RigidBody`] that does not move and is not affected by forces or collisions.
#[derive(Component, Reflect, Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[require(RigidBody)]
#[component(on_add = StaticBody::on_add, on_remove = StaticBody::on_remove)]
pub struct StaticBody;

impl StaticBody {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);
        let is_dynamic = entity_ref.contains::<DynamicBody>();
        let is_kinematic = entity_ref.contains::<KinematicBody>();

        // If the entity has a `DynamicBody` or `KinematicBody`, replace it.
        if is_dynamic {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a static body.
                    if entity_mut.contains::<StaticBody>() {
                        entity_mut.remove::<DynamicBody>();
                    }
                }
            });
        }
        if is_kinematic {
            world.commands().queue(move |world: &mut World| {
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    // Only remove the component if the entity is still a static body.
                    if entity_mut.contains::<StaticBody>() {
                        entity_mut.remove::<KinematicBody>();
                    }
                }
            });
        }
    }

    fn on_remove(mut world: DeferredWorld, ctx: HookContext) {
        let entity = ctx.entity;
        let entity_ref = world.entity(entity);

        // If the entity has no rigid body type left, remove `RigidBody`.
        if !entity_ref.contains::<DynamicBody>()
            && !entity_ref.contains::<KinematicBody>()
            && !entity_ref.contains::<StaticBody>()
        {
            world.commands().queue(move |world: &mut World| {
                // Only remove the component if the entity has no rigid body type left.
                if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
                    if !entity_mut.contains::<DynamicBody>()
                        && !entity_mut.contains::<KinematicBody>()
                        && !entity_mut.contains::<StaticBody>()
                    {
                        entity_mut.remove::<RigidBody>();
                    }
                }
            });
        }
    }
}

/// A query filter that selects rigid bodies that are neither disabled nor sleeping.
pub(crate) type RigidBodyActiveFilter = (Without<RigidBodyDisabled>, Without<Sleeping>);

/// A marker component that indicates that a [rigid body](RigidBody) is disabled
/// and should not participate in the simulation. Disables velocity, forces, contact response,
/// and attached joints.
///
/// This is useful for temporarily disabling a body without removing it from the world.
/// To re-enable the body, simply remove this component.
///
/// Note that this component does *not* disable collision detection or spatial queries for colliders
/// attached to the rigid body.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// #[derive(Component)]
/// pub struct Character;
///
/// /// Disables physics for all rigid body characters, for example during cutscenes.
/// fn disable_character_physics(
///     mut commands: Commands,
///     query: Query<Entity, (With<RigidBody>, With<Character>)>,
/// ) {
///     for entity in &query {
///         commands.entity(entity).insert(RigidBodyDisabled);
///     }
/// }
///
/// /// Enables physics for all rigid body characters.
/// fn enable_character_physics(
///     mut commands: Commands,
///     query: Query<Entity, (With<RigidBody>, With<Character>)>,
/// ) {
///     for entity in &query {
///         commands.entity(entity).remove::<RigidBodyDisabled>();
///     }
/// }
/// ```
///
/// # Related Components
///
/// - [`ColliderDisabled`]: Disables a collider.
/// - [`JointDisabled`]: Disables a joint constraint.
#[derive(Clone, Copy, Component, Reflect, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct RigidBodyDisabled;

/// Indicates that a [rigid body](RigidBody) is not simulated by the physics engine until woken up again.
/// This is done to improve performance and to help prevent small jitter that is typically present in collisions.
///
/// Bodies are marked as sleeping when their linear and angular velocity is below the [`SleepingThreshold`] for a time
/// indicated by [`DeactivationTime`]. A sleeping body is woken up when an active body interacts with it through
/// collisions or other constraints, or when gravity changes, or when the body's
/// position, rotation, velocity, or external forces are modified.
///
/// Sleeping can be disabled for specific entities with the [`SleepingDisabled`] component,
/// or for all entities by setting the [`SleepingThreshold`] to a negative value.
#[derive(Reflect, Clone, Copy, Component, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct Sleeping;

/// How long the velocity of the body has been below the [`SleepingThreshold`],
/// i.e. how long the body has been able to sleep.
///
/// See [`Sleeping`] for further information.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct TimeSleeping(pub Scalar);

impl TimeSleeping {
    /// Resets the time sleeping to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.0 = 0.0;
    }
}

/// Indicates that the body can not be deactivated by the physics engine. See [`Sleeping`] for information about sleeping.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct SleepingDisabled;

/// The linear velocity of a [rigid body](RigidBody), typically in meters per second.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn accelerate_linear(mut query: Query<&mut LinearVelocity>, time: Res<Time>) {
///     let delta_secs = time.delta_secs();
///     for mut linear_velocity in &mut query {
///         // Accelerate the entity towards +X at `2.0` units per second squared.
///         linear_velocity.x += 2.0 * delta_secs;
///     }
/// }
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// # Related Components
///
/// - [`AngularVelocity`]: The angular velocity of a body.
/// - [`LinearDamping`]: Reduces the linear velocity of a body over time, similar to air resistance.
/// - [`MaxLinearSpeed`]: Clamps the linear velocity of a body.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct LinearVelocity(pub Vector);

impl LinearVelocity {
    /// Zero linear velocity.
    pub const ZERO: LinearVelocity = LinearVelocity(Vector::ZERO);
}

/// The maximum linear speed of a [rigid body](RigidBody), clamping the [`LinearVelocity`],
/// typically in meters per second.
///
/// This can be useful for limiting how fast bodies can move, and can help control behavior and prevent instability.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a dynamic body with linear velocity clamped to `100.0` units per second.
/// fn setup(mut commands: Commands) {
///     commands.spawn((DynamicBody, MaxLinearSpeed(100.0)));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[doc(alias = "MaxLinearVelocity")]
pub struct MaxLinearSpeed(pub Scalar);

impl Default for MaxLinearSpeed {
    fn default() -> Self {
        Self(Scalar::INFINITY)
    }
}

/// The maximum angular speed of a [rigid body](RigidBody), clamping the [`AngularVelocity`],
/// in radians per second.
///
/// This can be useful for limiting how fast bodies can rotate, and can help control behavior and prevent instability.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a dynamic body with angular velocity clamped to `20.0` radians per second.
/// fn setup(mut commands: Commands) {
///     commands.spawn((DynamicBody, MaxAngularSpeed(20.0)));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
#[doc(alias = "MaxAngularVelocity")]
pub struct MaxAngularSpeed(pub Scalar);

impl Default for MaxAngularSpeed {
    fn default() -> Self {
        Self(Scalar::INFINITY)
    }
}

/// The angular velocity of a [rigid body](RigidBody) in radians per second.
/// Positive values will result in counterclockwise rotation.
///
/// # Example
///
/// ```
/// use avian2d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn accelerate_angular(mut query: Query<&mut AngularVelocity>, time: Res<Time>) {
///     let delta_secs = time.delta_secs();
///     for mut angular_velocity in &mut query {
///         // Accelerate rotation counterclockwise at `0.5` radians per second squared.
///         angular_velocity.0 += 0.5 * delta_secs;
///     }
/// }
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// # Related Components
///
/// - [`LinearVelocity`]: The linear velocity of a body.
/// - [`AngularDamping`]: Reduces the angular velocity of a body over time, similar to air resistance.
/// - [`MaxAngularSpeed`]: Clamps the angular velocity of a body.
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Deref, DerefMut, Component, Debug, Default, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularVelocity(pub Scalar);

/// The angular velocity of a [rigid body](RigidBody), represented as a rotation axis
/// multiplied by the angular speed in radians per second.
///
/// # Example
///
/// ```
/// use avian3d::prelude::*;
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn accelerate_angular(mut query: Query<&mut AngularVelocity>, time: Res<Time>) {
///     let delta_secs = time.delta_secs();
///     for mut angular_velocity in &mut query {
///         // Accelerate rotation about the Z axis at `0.5` radians per second squared.
///         angular_velocity.z += 0.5 * delta_secs;
///     }
/// }
/// # #[cfg(feature = "f64")]
/// # fn main() {}
/// ```
///
/// # Related Components
///
/// - [`LinearVelocity`]: The linear velocity of a body.
/// - [`AngularDamping`]: Reduces the angular velocity of a body over time, similar to air resistance.
/// - [`MaxAngularSpeed`]: Clamps the angular velocity of a body.
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularVelocity(pub Vector);

impl AngularVelocity {
    /// Zero angular velocity.
    #[cfg(feature = "2d")]
    pub const ZERO: AngularVelocity = AngularVelocity(0.0);
    /// Zero angular velocity.
    #[cfg(feature = "3d")]
    pub const ZERO: AngularVelocity = AngularVelocity(Vector::ZERO);
}

/// Controls how [gravity](Gravity) affects a specific [rigid body](RigidBody).
///
/// A gravity scale of `0.0` will disable gravity, while `2.0` will double the gravity.
/// Using a negative value will flip the direction of the gravity.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Spawn a dynamic body with `1.5` times the normal gravity.
/// fn setup(mut commands: Commands) {
///     commands.spawn((DynamicBody, GravityScale(1.5)));
/// }
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Deref, DerefMut, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct GravityScale(pub Scalar);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
    }
}

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing its
/// [linear velocity](LinearVelocity) each frame. This can be used to simulate air resistance.
///
/// The default linear damping coefficient is `0.0`, which corresponds to no damping.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((DynamicBody, LinearDamping(0.8)));
/// }
/// ```
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct LinearDamping(pub Scalar);

/// Automatically slows down a dynamic [rigid body](RigidBody), decreasing its
/// [angular velocity](AngularVelocity) each frame. This can be used to simulate air resistance.
///
/// The default angular damping coefficient is `0.0`, which corresponds to no damping.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((DynamicBody, AngularDamping(1.6)));
/// }
/// ```
#[derive(
    Component, Reflect, Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deref, DerefMut, From,
)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct AngularDamping(pub Scalar);

/// **Dominance** allows [dynamic rigid bodies](DynamicBody) to dominate
/// each other during physical interactions.
/// 
/// The body with a higher dominance acts as if it had infinite mass, and will be unaffected during
/// collisions and other interactions, while the other body will be affected normally.
/// 
/// The dominance must be between `-127` and `127`, and the default value is `0`.
/// Note that static and kinematic bodies will always have a higher dominance value
/// than dynamic bodies regardless of the value of this component.
/// 
/// # Example
/// 
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// // Player dominates all dynamic bodies with a dominance lower than `5`.
/// fn spawn_player(mut commands: Commands) {
///     commands.spawn((
///         DynamicBody,
///         Collider::capsule(0.4, 1.0),
///         Dominance(5),
///     ));
/// }
/// ```
#[rustfmt::skip]
#[derive(Component, Reflect, Debug, Clone, Copy, Default, Deref, DerefMut, From, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Dominance(pub i8);
