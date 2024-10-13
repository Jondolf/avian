#![allow(missing_docs)]

use crate::{prelude::*, utils::get_pos_translation};
use bevy::{
    ecs::query::QueryData,
    prelude::{Entity, Has, Ref},
};

/// A `WorldQuery` to make querying and modifying rigid bodies more convenient.
#[derive(QueryData)]
#[query_data(mutable)]
pub struct RigidBodyQuery {
    pub entity: Entity,
    pub rb: Ref<'static, RigidBody>,
    pub position: &'static mut Position,
    pub rotation: &'static mut Rotation,
    pub previous_rotation: &'static mut PreviousRotation,
    pub accumulated_translation: &'static mut AccumulatedTranslation,
    pub linear_velocity: &'static mut LinearVelocity,
    pub(crate) pre_solve_linear_velocity: &'static mut PreSolveLinearVelocity,
    pub angular_velocity: &'static mut AngularVelocity,
    pub(crate) pre_solve_angular_velocity: &'static mut PreSolveAngularVelocity,
    pub mass: &'static mut Mass,
    pub angular_inertia: &'static mut AngularInertia,
    #[cfg(feature = "3d")]
    pub global_angular_inertia: &'static mut GlobalAngularInertia,
    pub center_of_mass: &'static mut CenterOfMass,
    pub friction: &'static Friction,
    pub restitution: &'static Restitution,
    pub locked_axes: Option<&'static LockedAxes>,
    pub dominance: Option<&'static Dominance>,
    pub time_sleeping: &'static mut TimeSleeping,
    pub is_sleeping: Has<Sleeping>,
    pub is_sensor: Has<Sensor>,
}

impl<'w> RigidBodyQueryItem<'w> {
    /// Computes the velocity at the given `point` relative to the center of the body.
    pub fn velocity_at_point(&self, point: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.linear_velocity.0 + self.angular_velocity.0 * point.perp()
        }
        #[cfg(feature = "3d")]
        {
            self.linear_velocity.0 + self.angular_velocity.cross(point)
        }
    }

    /// Computes the effective inverse mass, taking into account any translation locking.
    pub fn effective_inverse_mass(&self) -> Vector {
        if !self.rb.is_dynamic() {
            return Vector::ZERO;
        }

        let mut inv_mass = Vector::splat(self.mass.inverse());

        if let Some(locked_axes) = self.locked_axes {
            inv_mass = locked_axes.apply_to_vec(inv_mass);
        }

        inv_mass
    }

    /// Returns the local angular inertia. If the rigid body is not dynamic, the returned angular inertia is infinite.
    pub fn angular_inertia(&self) -> AngularInertia {
        if self.rb.is_dynamic() {
            *self.angular_inertia
        } else {
            AngularInertia::INFINITY
        }
    }

    /// Computes the effective world-space angular inertia, taking into account any rotation locking.
    pub fn effective_global_angular_inertia(&self) -> AngularInertia {
        if !self.rb.is_dynamic() {
            return AngularInertia::INFINITY;
        }

        #[cfg(feature = "2d")]
        let mut angular_inertia = *self.angular_inertia;
        #[cfg(feature = "3d")]
        let mut angular_inertia = **self.global_angular_inertia;

        if let Some(locked_axes) = self.locked_axes {
            angular_inertia = locked_axes.apply_to_angular_inertia(angular_inertia);
        }

        angular_inertia
    }

    /// Returns the current position of the body. This is a sum of the [`Position`] and
    /// [`AccumulatedTranslation`] components.
    pub fn current_position(&self) -> Vector {
        self.position.0
            + get_pos_translation(
                &self.accumulated_translation,
                &self.previous_rotation,
                &self.rotation,
                &self.center_of_mass,
            )
    }

    /// Returns the [dominance](Dominance) of the body.
    ///
    /// If it isn't specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX` (`127`) is always returned instead.
    pub fn dominance(&self) -> i8 {
        if !self.rb.is_dynamic() {
            i8::MAX
        } else {
            self.dominance.map_or(0, |dominance| dominance.0)
        }
    }
}

impl<'w> RigidBodyQueryReadOnlyItem<'w> {
    /// Computes the velocity at the given `point` relative to the center of mass.
    pub fn velocity_at_point(&self, point: Vector) -> Vector {
        #[cfg(feature = "2d")]
        {
            self.linear_velocity.0 + self.angular_velocity.0 * point.perp()
        }
        #[cfg(feature = "3d")]
        {
            self.linear_velocity.0 + self.angular_velocity.cross(point)
        }
    }

    /// Returns the mass. If the rigid body is not dynamic, the returned mass is infinite.
    pub fn mass(&self) -> Mass {
        if self.rb.is_dynamic() {
            *self.mass
        } else {
            Mass::INFINITY
        }
    }

    /// Computes the effective inverse mass, taking into account any translation locking.
    pub fn effective_inverse_mass(&self) -> Vector {
        if !self.rb.is_dynamic() {
            return Vector::ZERO;
        }

        let mut inv_mass = Vector::splat(self.mass.inverse());

        if let Some(locked_axes) = self.locked_axes {
            inv_mass = locked_axes.apply_to_vec(inv_mass);
        }

        inv_mass
    }

    /// Returns the local angular inertia. If the rigid body is not dynamic, the returned angular inertia is infinite.
    pub fn angular_inertia(&self) -> AngularInertia {
        if self.rb.is_dynamic() {
            *self.angular_inertia
        } else {
            AngularInertia::INFINITY
        }
    }

    /// Computes the effective world-space angular inertia, taking into account any rotation locking.
    pub fn effective_global_angular_inertia(&self) -> AngularInertia {
        if !self.rb.is_dynamic() {
            return AngularInertia::INFINITY;
        }

        #[cfg(feature = "2d")]
        let mut angular_inertia = *self.angular_inertia;
        #[cfg(feature = "3d")]
        let mut angular_inertia = **self.global_angular_inertia;

        if let Some(locked_axes) = self.locked_axes {
            angular_inertia = locked_axes.apply_to_angular_inertia(angular_inertia);
        }

        angular_inertia
    }

    /// Returns the current position of the body. This is a sum of the [`Position`] and
    /// [`AccumulatedTranslation`] components.
    pub fn current_position(&self) -> Vector {
        self.position.0
            + get_pos_translation(
                self.accumulated_translation,
                self.previous_rotation,
                self.rotation,
                self.center_of_mass,
            )
    }

    /// Returns the [dominance](Dominance) of the body.
    ///
    /// If it isn't specified, the default of `0` is returned for dynamic bodies.
    /// For static and kinematic bodies, `i8::MAX` (`127`) is always returned instead.
    pub fn dominance(&self) -> i8 {
        if !self.rb.is_dynamic() {
            i8::MAX
        } else {
            self.dominance.map_or(0, |dominance| dominance.0)
        }
    }
}
