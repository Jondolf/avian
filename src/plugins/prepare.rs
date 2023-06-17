//! Performs necessary preparations and updates at the start of each physics frame. See [`PreparePlugin`].

use crate::{prelude::*, utils::make_isometry};
use bevy::prelude::*;

/// Performs necessary preparations and updates at the start of each physics frame. For example, [`ColliderAabb`]s and mass properties are updated.
pub struct PreparePlugin;

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.configure_set(ComponentInitSet.in_set(PhysicsSet::Prepare));
        app.add_systems(
            (init_rigid_bodies, init_mass_props, init_colliders).in_set(ComponentInitSet),
        );

        app.get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first")
            .add_systems(
                (update_aabb, update_mass_props)
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
    Option<&'static Pos>,
    Option<&'static Rot>,
    Option<&'static LinVel>,
    Option<&'static AngVel>,
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
            body.insert(PrevPos(pos.0));

            if let Some(ref mut transform) = transform {
                #[cfg(feature = "2d")]
                {
                    transform.translation = pos.extend(0.0).as_vec3_f32();
                }
                #[cfg(feature = "3d")]
                {
                    transform.translation = pos.as_vec3_f32();
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

            body.insert(Pos(translation));
            body.insert(PrevPos(translation));
        }

        if let Some(rot) = rot {
            body.insert(PrevRot(*rot));

            if let Some(mut transform) = transform {
                let q: Quaternion = (*rot).into();
                transform.rotation = q.as_quat_f32();
            }
        } else {
            let rotation =
                global_transform.map_or(Rot::default(), |t| t.compute_transform().rotation.into());
            body.insert(rotation);
            body.insert(PrevRot(rotation));
        }

        if lin_vel.is_none() {
            body.insert(LinVel::default());
        }
        body.insert(PreSolveLinVel::default());
        if ang_vel.is_none() {
            body.insert(AngVel::default());
        }
        body.insert(PreSolveAngVel::default());
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
    Option<&'static InvMass>,
    Option<&'static Inertia>,
    Option<&'static InvInertia>,
    Option<&'static LocalCom>,
);
type MassPropComponentsQueryFilter = Or<(Added<RigidBody>, Added<Collider>)>;

fn init_mass_props(
    mut commands: Commands,
    mass_props: Query<MassPropComponents, MassPropComponentsQueryFilter>,
) {
    for (entity, mass, inv_mass, inertia, inv_inertia, local_com) in &mass_props {
        let mut body = commands.entity(entity);

        if mass.is_none() {
            body.insert(Mass(inv_mass.map_or(0.0, |inv_mass| 1.0 / inv_mass.0)));
        }
        if inv_mass.is_none() {
            body.insert(InvMass(mass.map_or(0.0, |mass| 1.0 / mass.0)));
        }
        if inertia.is_none() {
            body.insert(inv_inertia.map_or(Inertia::ZERO, |inv_inertia| inv_inertia.inverse()));
        }
        if inv_inertia.is_none() {
            body.insert(inertia.map_or(InvInertia::ZERO, |inertia| inertia.inverse()));
        }
        if local_com.is_none() {
            body.insert(LocalCom::default());
        }
    }
}

type ColliderComponents = (
    Entity,
    &'static Collider,
    Option<&'static ColliderAabb>,
    Option<&'static CollidingEntities>,
    Option<&'static ColliderMassProperties>,
    Option<&'static PrevColliderMassProperties>,
);

fn init_colliders(mut commands: Commands, colliders: Query<ColliderComponents, Added<Collider>>) {
    for (entity, collider, aabb, colliding_entities, mass_props, prev_mass_props) in &colliders {
        let mut entity_commands = commands.entity(entity);

        if aabb.is_none() {
            entity_commands.insert(ColliderAabb::from_shape(collider.get_shape()));
        }
        if colliding_entities.is_none() {
            entity_commands.insert(CollidingEntities::default());
        }
        if mass_props.is_none() {
            entity_commands.insert(ColliderMassProperties::new_computed(collider, 1.0));
        }
        if prev_mass_props.is_none() {
            entity_commands.insert(PrevColliderMassProperties(ColliderMassProperties::ZERO));
        }
    }
}

type AABBChanged = Or<(Changed<Pos>, Changed<Rot>, Changed<LinVel>, Changed<AngVel>)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb(
    mut bodies: Query<(ColliderQuery, &Pos, &Rot, Option<&LinVel>, Option<&AngVel>), AABBChanged>,
    dt: Res<DeltaTime>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * dt.0;

    for (mut collider_query, pos, rot, lin_vel, ang_vel) in &mut bodies {
        let lin_vel_len = lin_vel.map_or(0.0, |v| v.length());

        #[cfg(feature = "2d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.abs());
        #[cfg(feature = "3d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.length());

        let computed_aabb = collider_query
            .collider
            .get_shape()
            .compute_aabb(&make_isometry(pos.0, rot));
        let half_extents = Vector::from(computed_aabb.half_extents());

        // Add a safety margin.
        let safety_margin = safety_margin_factor * (lin_vel_len + ang_vel_len);
        let extended_half_extents = half_extents + safety_margin;

        collider_query.aabb.mins.coords = (pos.0 - extended_half_extents).into();
        collider_query.aabb.maxs.coords = (pos.0 + extended_half_extents).into();
    }
}

type MassPropsChanged = Or<(
    Changed<Mass>,
    Changed<InvMass>,
    Changed<Inertia>,
    Changed<InvInertia>,
    Changed<Collider>,
    Changed<ColliderMassProperties>,
)>;

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_props(mut bodies: Query<(MassPropsQuery, Option<ColliderQuery>), MassPropsChanged>) {
    for (mut mass_props, collider) in &mut bodies {
        if mass_props.mass.is_changed() && mass_props.mass.0 >= Scalar::EPSILON {
            mass_props.inv_mass.0 = 1.0 / mass_props.mass.0;
        }

        if let Some(mut collider_query) = collider {
            // Subtract previous collider mass props from the body's mass props
            mass_props -= collider_query.prev_mass_props.0;

            // Update previous and current collider mass props
            collider_query.prev_mass_props.0 = *collider_query.mass_props;
            *collider_query.mass_props = ColliderMassProperties::new_computed(
                &collider_query.collider,
                collider_query.mass_props.density,
            );

            // Add new collider mass props to the body's mass props
            mass_props += *collider_query.mass_props;
        }

        if mass_props.mass.0 < Scalar::EPSILON {
            mass_props.mass.0 = Scalar::EPSILON;
        }
        if mass_props.inv_mass.0 < Scalar::EPSILON {
            mass_props.inv_mass.0 = Scalar::EPSILON;
        }
    }
}
