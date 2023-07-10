//! Rotation components.

use bevy::{math::DQuat, prelude::*};

use std::ops::{Add, AddAssign, Sub, SubAssign};

#[cfg(feature = "3d")]
use nalgebra::Matrix3x1;

use crate::prelude::*;

/// Radians
#[cfg(feature = "2d")]
pub(crate) type RotationValue = Scalar;
/// Quaternion
#[cfg(feature = "3d")]
pub(crate) type RotationValue = Quaternion;

/// The rotation of a body.
///
/// To speed up computation, the rotation is stored as the cosine and sine of the given angle in radians.
/// You should use the associated methods to create, access and modify rotations with normal radians or degrees.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_2d::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body rotated by 90 degrees
///     commands.spawn((RigidBody::Dynamic, Rotation::from_degrees(90.0)));
/// }
/// ```
#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug)]
#[reflect(Component)]
pub struct Rotation {
    /// The cosine of the rotation angle in radians.
    cos: Scalar,
    /// The sine of the rotation angle in radians.
    sin: Scalar,
}

/// The rotation of a body represented as a [`Quat`].
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// use bevy_xpbd_3d::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body rotated by 1.5 radians around the x axis
///     commands.spawn((RigidBody::Dynamic, Rotation(Quat::from_rotation_x(1.5))));
/// }
/// ```
#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct Rotation(pub Quaternion);

impl Rotation {
    /// Rotates the rotation by a 3D vector.
    #[cfg(feature = "2d")]
    pub fn rotate_vec3(&self, vec: crate::Vector3) -> crate::Vector3 {
        crate::Vector3::new(
            vec.x * self.cos() - vec.y * self.sin(),
            vec.x * self.sin() + vec.y * self.cos(),
            vec.z,
        )
    }
    /// Rotates the rotation by a 3D vector.
    #[cfg(feature = "3d")]
    pub fn rotate_vec3(&self, vec: Vector) -> Vector {
        self.0 * vec
    }
}

#[cfg(feature = "2d")]
impl Rotation {
    /// Zero rotation.
    pub const ZERO: Self = Self { cos: 1.0, sin: 0.0 };

    /// Returns the cosine of the rotation in radians.
    pub fn cos(&self) -> Scalar {
        self.cos
    }

    /// Returns the sine of the rotation in radians.
    pub fn sin(&self) -> Scalar {
        self.sin
    }

    /// Creates a [`Rotation`] from radians.
    pub fn from_radians(radians: Scalar) -> Self {
        Self {
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    /// Creates a [`Rotation`] from degrees.
    pub fn from_degrees(degrees: Scalar) -> Self {
        Self::from_radians(degrees.to_radians())
    }

    /// Returns the rotation in radians.
    pub fn as_radians(&self) -> Scalar {
        Scalar::atan2(self.sin(), self.cos())
    }

    /// Returns the rotation in degrees.
    pub fn as_degrees(&self) -> Scalar {
        self.as_radians().to_degrees()
    }

    /// Rotates the rotation by a given vector.
    pub fn rotate(&self, vec: Vector) -> Vector {
        Vector::new(
            vec.x * self.cos() - vec.y * self.sin(),
            vec.x * self.sin() + vec.y * self.cos(),
        )
    }

    /// Inverts the rotation.
    pub fn inverse(&self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    /// Multiplies the rotation by another rotation. This is equivalent to adding angles.
    pub fn mul(&self, rhs: Self) -> Self {
        Self {
            cos: self.cos * rhs.cos() - self.sin * rhs.sin(),
            sin: self.sin * rhs.cos() + self.cos * rhs.sin(),
        }
    }
}

#[cfg(feature = "3d")]
impl Rotation {
    /// Rotates the rotation by a given vector,
    pub fn rotate(&self, vec: Vector) -> Vector {
        self.0 * vec
    }

    /// Inverts the rotation.
    pub fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }
}

#[cfg(feature = "2d")]
impl Default for Rotation {
    fn default() -> Self {
        Self::ZERO
    }
}

#[cfg(feature = "2d")]
impl Add<Self> for Rotation {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        self.mul(rhs)
    }
}

#[cfg(feature = "3d")]
impl Add<Self> for Rotation {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Rotation(self.0 + rhs.0)
    }
}

impl AddAssign<Self> for Rotation {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

#[cfg(feature = "2d")]
impl Sub<Self> for Rotation {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        self.mul(rhs.inverse())
    }
}

#[cfg(feature = "3d")]
impl Sub<Self> for Rotation {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Rotation(self.0 - rhs.0)
    }
}

impl SubAssign<Self> for Rotation {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Scalar {
    fn from(rot: Rotation) -> Self {
        rot.as_radians()
    }
}

#[cfg(feature = "2d")]
impl From<Rotation> for Quaternion {
    fn from(rot: Rotation) -> Self {
        if rot.cos() < 0.0 {
            let t = 1.0 - rot.cos();
            let d = 1.0 / (t * 2.0).sqrt();
            let z = -rot.sin() * d;
            let w = t * d;
            Quaternion::from_xyzw(0.0, 0.0, z, w)
        } else {
            let t = 1.0 + rot.cos();
            let d = 1.0 / (t * 2.0).sqrt();
            let z = t * d;
            let w = -rot.sin() * d;
            Quaternion::from_xyzw(0.0, 0.0, z, w)
        }
    }
}

#[cfg(feature = "3d")]
impl From<Rotation> for Quaternion {
    fn from(rot: Rotation) -> Self {
        rot.0
    }
}

#[cfg(feature = "2d")]
impl From<Quat> for Rotation {
    fn from(quat: Quat) -> Self {
        let angle = quat.to_euler(EulerRot::XYZ).2;
        Self::from_radians(angle as Scalar)
    }
}

#[cfg(feature = "2d")]
impl From<DQuat> for Rotation {
    fn from(quat: DQuat) -> Self {
        let angle = quat.to_euler(EulerRot::XYZ).2;
        Self::from_radians(angle as Scalar)
    }
}

#[cfg(feature = "3d")]
impl From<Quat> for Rotation {
    fn from(quat: Quat) -> Self {
        Self(Quaternion::from_xyzw(
            quat.x as Scalar,
            quat.y as Scalar,
            quat.z as Scalar,
            quat.w as Scalar,
        ))
    }
}

#[cfg(feature = "3d")]
impl From<DQuat> for Rotation {
    fn from(quat: DQuat) -> Self {
        Self(Quaternion::from_xyzw(
            quat.x as Scalar,
            quat.y as Scalar,
            quat.z as Scalar,
            quat.w as Scalar,
        ))
    }
}

#[cfg(feature = "3d")]
impl From<Rotation> for Matrix3x1<Scalar> {
    fn from(rot: Rotation) -> Self {
        Matrix3x1::new(rot.x, rot.y, rot.z)
    }
}

/// The previous rotation of a body. See [`Rotation`].
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PreviousRotation(pub Rotation);
