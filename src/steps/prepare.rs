use crate::{prelude::*, utils::make_isometry};
use bevy::prelude::*;

pub struct PreparePlugin;

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app.get_schedule_mut(XpbdSchedule)
            .expect("add xpbd schedule first")
            .add_systems(
                (
                    update_sub_delta_time,
                    update_aabb,
                    update_mass_props.after(update_aabb),
                )
                    .in_set(PhysicsSet::Prepare),
            );
    }
}

type AABBChanged = Or<(Changed<Pos>, Changed<Rot>, Changed<LinVel>, Changed<AngVel>)>;

/// Updates the Axis-Aligned Bounding Boxes of all colliders. A safety margin will be added to account for sudden accelerations.
#[allow(clippy::type_complexity)]
fn update_aabb(
    mut bodies: Query<(ColliderQuery, &Pos, &Rot, Option<&LinVel>, Option<&AngVel>), AABBChanged>,
) {
    // Safety margin multiplier bigger than DELTA_TIME to account for sudden accelerations
    let safety_margin_factor = 2.0 * DELTA_TIME;

    for (mut collider, pos, rot, lin_vel, ang_vel) in &mut bodies {
        let lin_vel_len = lin_vel.map_or(0.0, |v| v.length());

        #[cfg(feature = "2d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.abs());
        #[cfg(feature = "3d")]
        let ang_vel_len = ang_vel.map_or(0.0, |v| v.length());

        let computed_aabb = collider.shape.compute_aabb(&make_isometry(pos.0, rot));
        let half_extents = Vector::from(computed_aabb.half_extents());

        // Add a safety margin.
        let safety_margin = safety_margin_factor * (lin_vel_len + ang_vel_len);
        let extended_half_extents = half_extents + safety_margin;

        collider.aabb.mins.coords = (pos.0 - extended_half_extents).into();
        collider.aabb.maxs.coords = (pos.0 + extended_half_extents).into();
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
