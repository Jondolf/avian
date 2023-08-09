//! Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
//! and [colliders](Collider) and updates components.
//!
//! See [`PreparePlugin`].

use crate::prelude::*;
use bevy::prelude::*;

/// Runs systems at the start of each physics frame; initializes [rigid bodies](RigidBody)
/// and [colliders](Collider) and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Adds missing collider components for entities with a [`Collider`] component
/// - Adds missing mass properties for entities with a [`RigidBody`] or [`Collider`] component
/// - Updates mass properties and adds [`ColliderMassProperties`] on top of the existing mass properties
/// - Clamps restitution coefficients between 0 and 1
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin {
    schedule: Box<dyn ScheduleLabel>,
}

impl PreparePlugin {
    /// Creates a [`PreparePlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `PostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: Box::new(schedule),
        }
    }
}

impl Default for PreparePlugin {
    fn default() -> Self {
        Self::new(PostUpdate)
    }
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            self.schedule.dyn_clone(),
            (
                (
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                    init_rigid_bodies,
                )
                    .chain()
                    .run_if(any_new_rigid_bodies),
                init_mass_properties,
                init_colliders,
                update_mass_properties,
                clamp_restitution,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );
    }
}

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
    Option<&'static ExternalImpulse>,
    Option<&'static ExternalAngularImpulse>,
    Option<&'static Restitution>,
    Option<&'static Friction>,
    Option<&'static TimeSleeping>,
);

fn any_new_rigid_bodies(query: Query<(), Added<RigidBody>>) -> bool {
    !query.is_empty()
}

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
        impulse,
        angular_impulse,
        restitution,
        friction,
        time_sleeping,
    ) in &mut bodies
    {
        let mut body = commands.entity(entity);
        body.insert(AccumulatedTranslation(Vector::ZERO));

        if let Some(pos) = pos {
            body.insert(PreviousPosition(pos.0));

            if let Some(ref mut transform) = transform {
                #[cfg(feature = "2d")]
                {
                    transform.translation = pos.as_f32().extend(transform.translation.z);
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
        if impulse.is_none() {
            body.insert(ExternalImpulse::default());
        }
        if angular_impulse.is_none() {
            body.insert(ExternalAngularImpulse::default());
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
    // Use transform as default position and rotation if no components for them found
    Option<&'static mut Transform>,
    Option<&'static GlobalTransform>,
    Option<&'static Position>,
    Option<&'static Rotation>,
    &'static Collider,
    Option<&'static ColliderAabb>,
    Option<&'static CollidingEntities>,
    Option<&'static ColliderMassProperties>,
    Option<&'static PreviousColliderMassProperties>,
);

fn init_colliders(
    mut commands: Commands,
    mut colliders: Query<ColliderComponents, Added<Collider>>,
) {
    for (
        entity,
        mut transform,
        global_transform,
        pos,
        rot,
        collider,
        aabb,
        colliding_entities,
        mass_properties,
        previous_mass_properties,
    ) in &mut colliders
    {
        let mut entity_commands = commands.entity(entity);

        if let Some(pos) = pos {
            if let Some(ref mut transform) = transform {
                #[cfg(feature = "2d")]
                {
                    transform.translation = pos.as_f32().extend(transform.translation.z);
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

            entity_commands.insert(Position(translation));
        }

        if let Some(rot) = rot {
            if let Some(mut transform) = transform {
                let q: Quaternion = (*rot).into();
                transform.rotation = q.as_f32();
            }
        } else {
            let rotation = global_transform.map_or(Rotation::default(), |t| {
                t.compute_transform().rotation.into()
            });
            entity_commands.insert(rotation);
        }

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

type MassPropertiesComponents = (
    Entity,
    Option<&'static RigidBody>,
    MassPropertiesQuery,
    Option<&'static Collider>,
    Option<&'static mut ColliderMassProperties>,
    Option<&'static mut PreviousColliderMassProperties>,
);

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
fn update_mass_properties(mut bodies: Query<MassPropertiesComponents, MassPropertiesChanged>) {
    for (
        entity,
        rb,
        mut mass_properties,
        collider,
        collider_mass_properties,
        previous_collider_mass_properties,
    ) in &mut bodies
    {
        if mass_properties.mass.is_changed() && mass_properties.mass.0 >= Scalar::EPSILON {
            mass_properties.inverse_mass.0 = 1.0 / mass_properties.mass.0;
        }

        if let Some(collider) = collider {
            let Some(mut collider_mass_properties) = collider_mass_properties else {
                continue;
            };
            let Some(mut previous_collider_mass_properties) = previous_collider_mass_properties
            else {
                continue;
            };

            // Subtract previous collider mass props from the body's mass props
            mass_properties -= previous_collider_mass_properties.0;

            // Update previous and current collider mass props
            previous_collider_mass_properties.0 = *collider_mass_properties;
            *collider_mass_properties =
                ColliderMassProperties::new_computed(collider, collider_mass_properties.density);

            // Add new collider mass props to the body's mass props
            mass_properties += *collider_mass_properties;
        }

        // Warn about dynamic bodies with no mass or inertia
        if let Some(rb) = rb {
            let is_mass_valid =
                mass_properties.mass.is_finite() && mass_properties.mass.0 >= Scalar::EPSILON;
            #[cfg(feature = "2d")]
            let is_inertia_valid =
                mass_properties.inertia.is_finite() && mass_properties.inertia.0 >= Scalar::EPSILON;
            #[cfg(feature = "3d")]
            let is_inertia_valid =
                mass_properties.inertia.is_finite() && *mass_properties.inertia != Inertia::ZERO;
            if rb.is_dynamic() && !(is_mass_valid && is_inertia_valid) {
                warn!(
                    "Dynamic rigid body {:?} has no mass or inertia. This can cause NaN values. Consider adding a `MassPropertiesBundle` or a `Collider` with mass.",
                    entity
                );
            }
        }
    }
}

fn clamp_restitution(mut query: Query<&mut Restitution, Changed<Restitution>>) {
    for mut restitution in &mut query {
        restitution.coefficient = restitution.coefficient.clamp(0.0, 1.0);
    }
}
