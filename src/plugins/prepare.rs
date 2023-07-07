//! Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
//! and [colliders](Collider) and updates components.
//!
//! See [`PreparePlugin`].

use crate::{prelude::*, utils::make_isometry};
use bevy::prelude::*;

/// Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
/// and [colliders](Collider) and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Adds missing collider components for entities with a [`Collider`] component
/// - Updates [AABBs](ColliderAabb)
/// - Updates mass properties and adds [`ColliderMassProperties`] on top of the existing mass properties
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin;

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.configure_set(ComponentInitSet.in_set(PhysicsSet::Prepare));
        app.add_systems(
            (init_rigid_bodies, init_mass_properties, init_colliders).in_set(ComponentInitSet),
        );

        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                (update_aabb, update_mass_properties)
                    .chain()
                    .after(ComponentInitSet)
                    .in_set(PhysicsSet::Prepare),
            );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct ComponentInitSet;

type RigidBodyComponents = (
    Entity,
    // Use transform as default position and rotation if no components for them found
    Option<&'static mut Transform>,
    Option<&'static GlobalTransform>,
    Option<&'static Position>,
    Option<&'static Rotation>,
    Option<&'static LinearVelocity>,
    Option<&'static AngularVelocity>,
    Option<&'static ExternalForce>,
    Option<&'static ExternalTorque>,
    Option<&'static Restitution>,
    Option<&'static Friction>,
    Option<&'static TimeSleeping>,
);

fn init_rigid_bodies(
    mut commands: Commands,
    mut bodies: Query<RigidBodyComponents, Added<RigidBody>>,
) {
    for (
        entity,
        mut transform,
        global_transform,
        pos,
        rot,
        lin_vel,
        ang_vel,
        force,
        torque,
        restitution,
        friction,
        time_sleeping,
    ) in &mut bodies
    {
        let mut body = commands.entity(entity);

        if let Some(pos) = pos {
            body.insert(PreviousPosition(pos.0));

            if let Some(ref mut transform) = transform {
                #[cfg(feature = "2d")]
                {
                    transform.translation = pos.extend(0.0).as_f32();
                }
                #[cfg(feature = "3d")]
                {
                    transform.translation = pos.as_f32();
                }
            }
        } else {
            let translation;
            #[cfg(feature = "2d")]
            {
                translation = global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(t.translation().x as Scalar, t.translation().y as Scalar)
                });
            }
            #[cfg(feature = "3d")]
            {
                translation = global_transform.as_ref().map_or(Vector::ZERO, |t| {
                    Vector::new(
                        t.translation().x as Scalar,
                        t.translation().y as Scalar,
                        t.translation().z as Scalar,
                    )
                });
            }

            body.insert(Position(translation));
            body.insert(PreviousPosition(translation));
        }

        if let Some(rot) = rot {
            body.insert(PreviousRotation(*rot));

            if let Some(mut transform) = transform {
                let q: Quaternion = (*rot).into();
                transform.rotation = q.as_f32();
            }
        } else {
            let rotation = global_transform.map_or(Rotation::default(), |t| {
                t.compute_transform().rotation.into()
            });
            body.insert(rotation);
            body.insert(PreviousRotation(rotation));
        }

        if lin_vel.is_none() {
            body.insert(LinearVelocity::default());
        }
        body.insert(PreSolveLinearVelocity::default());
        if ang_vel.is_none() {
            body.insert(AngularVelocity::default());
        }
        body.insert(PreSolveAngularVelocity::default());
        if force.is_none() {
            body.insert(ExternalForce::default());
        }
        if torque.is_none() {
            body.insert(ExternalTorque::default());
        }
        if restitution.is_none() {
            body.insert(Restitution::default());
        }
        if friction.is_none() {
            body.insert(Friction::default());
        }
        if time_sleeping.is_none() {
            body.insert(TimeSleeping::default());
        }
    }
}

type MassPropComponents = (
    Entity,
    Option<&'static Mass>,
    Option<&'static InverseMass>,
    Option<&'static Inertia>,
    Option<&'static InverseInertia>,
    Option<&'static CenterOfMass>,
);
type MassPropComponentsQueryFilter = Or<(Added<RigidBody>, Added<Collider>)>;

fn init_mass_properties(
    mut commands: Commands,
    mass_properties: Query<MassPropComponents, MassPropComponentsQueryFilter>,
) {
    for (entity, mass, inverse_mass, inertia, inverse_inertia, center_of_mass) in &mass_properties {
        let mut body = commands.entity(entity);

        if mass.is_none() {
            body.insert(Mass(
                inverse_mass.map_or(0.0, |inverse_mass| 1.0 / inverse_mass.0),
            ));
        }
        if inverse_mass.is_none() {
            body.insert(InverseMass(mass.map_or(0.0, |mass| 1.0 / mass.0)));
        }
        if inertia.is_none() {
            body.insert(
                inverse_inertia.map_or(Inertia::ZERO, |inverse_inertia| inverse_inertia.inverse()),
            );
        }
        if inverse_inertia.is_none() {
            body.insert(inertia.map_or(InverseInertia::ZERO, |inertia| inertia.inverse()));
        }
        if center_of_mass.is_none() {
            body.insert(CenterOfMass::default());
        }
    }
}

type ColliderComponents = (
    Entity,
    &'static Collider,
    Option<&'static ColliderAabb>,
    Option<&'static CollidingEntities>,
    Option<&'static ColliderMassProperties>,
    Option<&'static PreviousColliderMassProperties>,
);

fn init_colliders(mut commands: Commands, colliders: Query<ColliderComponents, Added<Collider>>) {
    for (entity, collider, aabb, colliding_entities, mass_properties, previous_mass_properties) in
        &colliders
    {
        let mut entity_commands = commands.entity(entity);

        if aabb.is_none() {
            entity_commands.insert(ColliderAabb::from_shape(collider.get_shape()));
        }
        if colliding_entities.is_none() {
            entity_commands.insert(CollidingEntities::default());
        }
        if mass_properties.is_none() {
            entity_commands.insert(ColliderMassProperties::new_computed(collider, 1.0));
        }
        if previous_mass_properties.is_none() {
            entity_commands.insert(PreviousColliderMassProperties(ColliderMassProperties::ZERO));
        }
    }
}

type AABBChanged = Or<(
    Changed<Position>,
    Changed<Rotation>,
    Changed<LinearVelocity>,
    Changed<AngularVelocity>,
)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb(
    mut bodies: Query<
        (
            ColliderQuery,
            &Position,
            &Rotation,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        AABBChanged,
    >,
    dt: Res<DeltaTime>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.0;

    for (mut collider_query, pos, rot, lin_vel, ang_vel) in &mut bodies {
        let lin_vel_len = lin_vel.map_or(0.0, |v| v.length());

        #[cfg(feature = "2d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.0.abs());
        #[cfg(feature = "3d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.0.length());

        let computed_aabb = collider_query
            .collider
            .get_shape()
            .compute_aabb(&make_isometry(pos.0, rot));
        let half_extents = Vector::from(computed_aabb.half_extents());
        let center = Vector::from(computed_aabb.center());

        // Add a safety margin.
        let safety_margin = safety_margin_factor * (lin_vel_len + ang_vel_len);
        let extended_half_extents = half_extents + safety_margin;

        collider_query.aabb.mins.coords = (center - extended_half_extents).into();
        collider_query.aabb.maxs.coords = (center + extended_half_extents).into();
    }
}

type MassPropertiesChanged = Or<(
    Changed<Mass>,
    Changed<InverseMass>,
    Changed<Inertia>,
    Changed<InverseInertia>,
    Changed<Collider>,
    Changed<ColliderMassProperties>,
)>;

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_properties(
    mut bodies: Query<(MassPropertiesQuery, Option<ColliderQuery>), MassPropertiesChanged>,
) {
    for (mut mass_properties, collider) in &mut bodies {
        if mass_properties.mass.is_changed() && mass_properties.mass.0 >= Scalar::EPSILON {
            mass_properties.inverse_mass.0 = 1.0 / mass_properties.mass.0;
        }

        if let Some(mut collider_query) = collider {
            // Subtract previous collider mass props from the body's mass props
            mass_properties -= collider_query.previous_mass_properties.0;

            // Update previous and current collider mass props
            collider_query.previous_mass_properties.0 = *collider_query.mass_properties;
            *collider_query.mass_properties = ColliderMassProperties::new_computed(
                &collider_query.collider,
                collider_query.mass_properties.density,
            );

            // Add new collider mass props to the body's mass props
            mass_properties += *collider_query.mass_properties;
        }

        if mass_properties.mass.0 < Scalar::EPSILON {
            mass_properties.mass.0 = Scalar::EPSILON;
        }
        if mass_properties.inverse_mass.0 < Scalar::EPSILON {
            mass_properties.inverse_mass.0 = Scalar::EPSILON;
        }
    }
}
