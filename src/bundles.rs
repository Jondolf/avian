use crate::{components::*, Scalar};

use bevy::prelude::*;

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
}

impl RigidBodyBundle {
    pub fn new_dynamic() -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            ..default()
        }
    }

    pub fn new_static() -> Self {
        Self {
            rigid_body: RigidBody::Static,
            ..default()
        }
    }

    pub fn new_kinematic() -> Self {
        Self {
            rigid_body: RigidBody::Kinematic,
            ..default()
        }
    }

    pub fn with_pos(self, pos: impl Into<Pos>) -> Self {
        let pos = pos.into();
        Self { pos, ..self }
    }

    pub fn with_rot(self, rot: impl Into<Rot>) -> Self {
        let rot = rot.into();
        Self { rot, ..self }
    }

    pub fn with_lin_vel(self, lin_vel: impl Into<LinVel>) -> Self {
        let lin_vel = lin_vel.into();
        Self { lin_vel, ..self }
    }

    pub fn with_ang_vel(self, ang_vel: impl Into<AngVel>) -> Self {
        let ang_vel = ang_vel.into();
        Self { ang_vel, ..self }
    }

    /// Computes the mass properties that a [`Collider`] would have with a given density, and adds those to the body.
    pub fn with_mass_props_from_shape(self, shape: &Shape, density: Scalar) -> Self {
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

#[derive(Bundle, Default)]
pub struct ColliderBundle {
    collider_shape: ColliderShape,
    collider_aabb: ColliderAabb,
    pub mass_props: ColliderMassProperties,
    pub(crate) prev_mass_props: PrevColliderMassProperties,
}

impl ColliderBundle {
    /// Creates a new [`ColliderBundle`] from a given [`ColliderShape`] and density.
    pub fn new(shape: &Shape, density: Scalar) -> Self {
        let aabb = ColliderAabb::from_shape(shape);
        let mass_props = ColliderMassProperties::from_shape_and_density(shape, density);

        Self {
            collider_shape: ColliderShape(shape.to_owned()),
            collider_aabb: aabb,
            mass_props,
            prev_mass_props: PrevColliderMassProperties(ColliderMassProperties::ZERO),
        }
    }

    pub fn update_mass_props(&mut self) {
        self.mass_props = ColliderMassProperties::from_shape_and_density(
            &self.collider_shape.0,
            self.mass_props.density,
        );
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
