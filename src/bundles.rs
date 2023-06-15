//! Bundles with physics components.

use crate::{components::*, Scalar};

use bevy::prelude::*;

/// A bundle with all of the components that a rigid body needs. The associated builder methods can be used to configure the components.
///
/// ## Example
///
/// Below is an example of creating a dynamic rigid body with an initial position, rotation, linear velocity, angular velocity, and mass properties.
///
/// ```ignore
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// let mut app = App::new();
///
/// app.add_plugins(DefaultPlugins).add_plugins(PhysicsPlugins).run();
///
/// app.world.spawn(
///     RigidBodyBundle::new_dynamic()
///         .with_pos(Vec3::Y * 4.0)
///         .with_rot(Quat::from_rotation_x(2.1))
///         .with_lin_vel(Vec3::new(0.0, 4.5, 0.0))
///         .with_ang_vel(Vec3::new(2.5, 3.4, 1.6))
///         .with_computed_mass_props(&Shape::cuboid(0.5, 0.5, 0.5), 1.0),
/// );
/// ```
#[derive(Bundle, Default)]
pub struct RigidBodyBundle {
    pub rigid_body: RigidBody,

    pub pos: Pos,
    pub prev_pos: PrevPos,

    pub rot: Rot,
    pub prev_rot: PrevRot,

    pub lin_vel: LinVel,
    pub pre_solve_lin_vel: PreSolveLinVel,

    pub ang_vel: AngVel,
    pub pre_solve_ang_vel: PreSolveAngVel,

    pub external_force: ExternalForce,
    pub external_torque: ExternalTorque,

    pub restitution: Restitution,
    pub friction: Friction,

    pub mass: Mass,
    pub inv_mass: InvMass,
    pub inertia: Inertia,
    pub inv_inertia: InvInertia,
    pub local_center_of_mass: LocalCom,

    pub time_sleeping: TimeSleeping,
}

impl RigidBodyBundle {
    /// Creates a dynamic rigid body. See [`RigidBody::Dynamic`].
    pub fn new_dynamic() -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            ..default()
        }
    }

    /// Creates a static rigid body. See [`RigidBody::Static`].
    pub fn new_static() -> Self {
        Self {
            rigid_body: RigidBody::Static,
            ..default()
        }
    }

    /// Creates a kinematic rigid body. See [`RigidBody::Kinematic`].
    pub fn new_kinematic() -> Self {
        Self {
            rigid_body: RigidBody::Kinematic,
            ..default()
        }
    }

    /// Sets the position of a rigid body. See [`Pos`].
    pub fn with_pos(self, pos: impl Into<Pos>) -> Self {
        let pos = pos.into();
        Self { pos, ..self }
    }

    /// Sets the rotation of a rigid body. See [`Rot`].
    pub fn with_rot(self, rot: impl Into<Rot>) -> Self {
        let rot = rot.into();
        Self { rot, ..self }
    }

    /// Sets the linear velocity of a rigid body. See [`LinVel`].
    pub fn with_lin_vel(self, lin_vel: impl Into<LinVel>) -> Self {
        let lin_vel = lin_vel.into();
        Self { lin_vel, ..self }
    }

    /// Sets the angular velocity of a rigid body. See [`AngVel`].
    pub fn with_ang_vel(self, ang_vel: impl Into<AngVel>) -> Self {
        let ang_vel = ang_vel.into();
        Self { ang_vel, ..self }
    }

    /// Sets the restitution of a rigid body. See [`Restitution`].
    pub fn with_restitution(self, restitution: impl Into<Restitution>) -> Self {
        let restitution = restitution.into();
        Self {
            restitution,
            ..self
        }
    }

    /// Sets the friction of a rigid body. See [`Friction`].
    pub fn with_friction(self, friction: impl Into<Friction>) -> Self {
        let friction = friction.into();
        Self { friction, ..self }
    }

    /// Sets the mass of a rigid body. See [`Mass`].
    pub fn with_mass(self, mass: impl Into<Mass>) -> Self {
        let mass = mass.into();
        let inv_mass = InvMass(1.0 / mass.0);
        Self {
            mass,
            inv_mass,
            ..self
        }
    }

    /// Sets the moment of inertia of a rigid body. See [`Inertia`].
    pub fn with_inertia(self, inertia: impl Into<Inertia>) -> Self {
        let inertia = inertia.into();
        let inv_inertia = inertia.inverse();
        Self {
            inertia,
            inv_inertia,
            ..self
        }
    }

    /// Sets the local center of mass of a rigid body. See [`LocalCom`].
    pub fn with_local_center_of_mass(self, local_center_of_mass: impl Into<LocalCom>) -> Self {
        let local_center_of_mass = local_center_of_mass.into();
        Self {
            local_center_of_mass,
            ..self
        }
    }

    /// Sets the mass properties of a rigid body by computing the [`ColliderMassProperties`] that a given [`ColliderShape`] would have with a given density.
    ///
    /// For the affected mass properties, see [`Mass`], [`InvMass`], [`Inertia`], [`InvInertia`] and [`LocalCom`].
    pub fn with_computed_mass_props(self, shape: &Shape, density: Scalar) -> Self {
        let ColliderMassProperties {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
            ..
        } = ColliderMassProperties::from_shape_and_density(shape, density);

        Self {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            local_center_of_mass,
            ..self
        }
    }
}

/// A bundle for the components associated with a collider.
/// This includes the [`ColliderShape`], [`ColliderAabb`] and [`ColliderMassProperties`]
#[derive(Bundle, Default)]
pub struct ColliderBundle {
    pub collider_shape: ColliderShape,
    pub collider_aabb: ColliderAabb,
    pub colliding_entities: CollidingEntities,
    pub mass_props: ColliderMassProperties,
    pub(crate) prev_mass_props: PrevColliderMassProperties,
}

impl ColliderBundle {
    /// Creates a new [`ColliderBundle`] from a given [`Shape`] with a default density of 1.
    pub fn new(shape: &Shape) -> Self {
        Self {
            collider_shape: ColliderShape(shape.to_owned()),
            collider_aabb: ColliderAabb::from_shape(shape),
            colliding_entities: CollidingEntities::default(),
            mass_props: ColliderMassProperties::from_shape_and_density(shape, 1.0),
            prev_mass_props: PrevColliderMassProperties(ColliderMassProperties::ZERO),
        }
    }
    /// Sets the collider's mass properties computed from a given density.
    pub fn with_density(self, density: Scalar) -> Self {
        let shape = &self.collider_shape;
        Self {
            mass_props: ColliderMassProperties::from_shape_and_density(shape, density),
            prev_mass_props: PrevColliderMassProperties(self.mass_props),
            ..self
        }
    }
}

#[cfg(test)]
mod test {
    use crate::prelude::*;
    use approx::assert_relative_eq;

    #[cfg(feature = "2d")]
    #[test]
    fn body_builder_accepts_vec_2d() {
        let body = RigidBodyBundle::new_dynamic()
            .with_ang_vel(1.)
            .with_lin_vel(Vector::new(2., 3.))
            .with_pos(Vector::new(4., 5.))
            .with_rot(Rot::from_radians(0.123));

        assert_relative_eq!(body.ang_vel.0, 1.);
        assert_relative_eq!(body.lin_vel.0, Vector::new(2., 3.));
        assert_relative_eq!(body.pos.0, Vector::new(4., 5.));
        assert_relative_eq!(body.rot.as_radians(), 0.123);
    }

    #[cfg(feature = "3d")]
    #[test]
    fn body_builder_accepts_vec_3d() {
        let body = RigidBodyBundle::new_dynamic()
            .with_ang_vel(Vec3::X)
            .with_lin_vel(Vec3::new(2., 3., 4.))
            .with_pos(Vector::new(5., 6., 7.))
            .with_rot(Quaternion::from_axis_angle(Vector::X, 0.123));

        assert_relative_eq!(body.ang_vel.0, Vector::X);
        assert_relative_eq!(body.lin_vel.0, Vector::new(2., 3., 4.));
        assert_relative_eq!(body.pos.0, Vector::new(5., 6., 7.));
        assert_relative_eq!(body.rot.0, Quaternion::from_axis_angle(Vector::X, 0.123));
    }
}
