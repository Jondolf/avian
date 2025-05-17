//! Forces, torques, linear impulses, and angular impulses
//! that can be applied to dynamic rigid bodies.

mod plugin;
mod system_param;

pub use plugin::{ForcePlugin, ForceSet};
pub use system_param::{EntityForces, ForceHelper};

use crate::prelude::*;
use bevy::prelude::*;

#[cfg(feature = "2d")]
pub(crate) type Torque = Scalar;

#[cfg(feature = "3d")]
pub(crate) type Torque = Vector;

#[cfg(feature = "2d")]
pub(crate) trait FloatZero {
    const ZERO: Self;
}

#[cfg(feature = "2d")]
impl FloatZero for Scalar {
    const ZERO: Self = 0.0;
}

#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedWorldForces)]
pub struct ConstantForce(pub Vector);

#[cfg(feature = "2d")]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedWorldForces)]
pub struct ConstantTorque(pub Scalar);

#[cfg(feature = "3d")]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedWorldForces)]
pub struct ConstantTorque(pub Vector);

#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedLocalForces)]
pub struct ConstantLocalForce(pub Vector);

#[cfg(feature = "2d")]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedLocalForces)]
pub struct ConstantLocalTorque(pub Scalar);

#[cfg(feature = "3d")]
#[derive(Component, Clone, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
#[require(AccumulatedLocalForces)]
pub struct ConstantLocalTorque(pub Vector);

// TODO: Should accumulated forces and accelerations be `SparseSet` components?

/// A component with the user-applied world forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero world forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedWorldForces {
    pub force: Vector,
    pub torque: Torque,
}

/// A component with the user-applied local forces and torques
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local forces have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalForces {
    pub force: Vector,
    pub torque: Torque,
}

/// A component with the user-applied local acceleration
/// accumulated for a rigid body before the physics step.
///
/// Only entities with non-zero local acceleration have this component.
/// It is added and removed automatically to reduce unnecessary work.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct AccumulatedLocalAcceleration {
    pub linear: Vector,
    #[cfg(feature = "3d")]
    pub angular: Vector,
}
