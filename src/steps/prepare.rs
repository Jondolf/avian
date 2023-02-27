use crate::prelude::*;
use bevy::prelude::*;

pub struct PreparePlugin;

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.add_system_set_to_stage(
            FixedUpdateStage,
            SystemSet::new()
                .before(PhysicsStep::BroadPhase)
                .label(PhysicsStep::Prepare)
                .with_run_criteria(first_substep)
                .with_system(sync_transforms)
                .with_system(update_sub_delta_time)
                .with_system(update_aabb)
                .with_system(update_mass_props),
        );
    }
}

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "2d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.extend(0.0);
        transform.rotation = (*rot).into();
    }
}

/// Copies positions from the physics world to bevy Transforms
#[cfg(feature = "3d")]
fn sync_transforms(mut bodies: Query<(&mut Transform, &Pos, &Rot)>) {
    for (mut transform, pos, rot) in &mut bodies {
        transform.translation = pos.0;
        transform.rotation = rot.0;
    }
}

type AABBChanged = Or<(Changed<Pos>, Changed<Rot>, Changed<LinVel>)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
fn update_aabb(mut bodies: Query<(ColliderQuery, &Pos, &Rot, Option<&LinVel>), AABBChanged>) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * DELTA_TIME;

    for (mut collider, pos, rot, lin_vel) in &mut bodies {
        let lin_vel = lin_vel.map_or(Vector::ZERO, |v| v.0);

        let half_extents = Vector::from(
            collider
                .shape
                .compute_aabb(&Isometry::new(pos.0.into(), (*rot).into()))
                .half_extents(),
        );
        // Add a safety margin.
        let scaled_half_extents = (half_extents + safety_margin_factor * lin_vel.length()) * 1.0;

        collider.aabb.mins.coords = (pos.0 - scaled_half_extents).into();
        collider.aabb.maxs.coords = (pos.0 + scaled_half_extents).into();
    }
}

fn update_sub_delta_time(substeps: Res<NumSubsteps>, mut sub_dt: ResMut<SubDeltaTime>) {
    sub_dt.0 = DELTA_TIME / substeps.0 as f32;
}

type MassPropsChanged = Or<(
    Changed<Mass>,
    Changed<InvMass>,
    Changed<Inertia>,
    Changed<InvInertia>,
    Changed<ColliderShape>,
    Changed<ColliderMassProperties>,
)>;

/// Updates each body's mass properties whenever their dependant mass properties or the body's [`Collider`] change.
///
/// Also updates the collider's mass properties if the body has a collider.
fn update_mass_props(
    mut bodies: Query<(MassPropsQueryMut, Option<ColliderQuery>), MassPropsChanged>,
) {
    for (mut mass_props, collider) in &mut bodies {
        if mass_props.mass.is_changed() && mass_props.mass.0 >= f32::EPSILON {
            mass_props.inv_mass.0 = 1.0 / mass_props.mass.0;
        }

        if let Some(mut collider) = collider {
            // Subtract previous collider mass props from the body's mass props
            mass_props -= collider.prev_mass_props.0;

            // Update previous and current collider mass props
            collider.prev_mass_props.0 = *collider.mass_props;
            *collider.mass_props = ColliderMassProperties::from_shape_and_density(
                &collider.shape.0,
                collider.mass_props.density,
            );

            // Add new collider mass props to the body's mass props
            mass_props += *collider.mass_props;
        }

        if mass_props.mass.0 < f32::EPSILON {
            mass_props.mass.0 = f32::EPSILON;
        }
        if mass_props.inv_mass.0 < f32::EPSILON {
            mass_props.inv_mass.0 = f32::EPSILON;
        }
    }
}
