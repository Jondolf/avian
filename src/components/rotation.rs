use bevy::prelude::*;

use std::ops::{Add, AddAssign, Sub, SubAssign};

#[cfg(feature = "3d")]
use nalgebra::Matrix3x1;

use crate::Vector;

#[cfg(feature = "2d")]
#[derive(Reflect, Clone, Copy, Component, Debug)]
#[reflect(Component)]
pub struct Rot {
    pub cos: f32,
    pub sin: f32,
}

#[cfg(feature = "3d")]
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, derive_more::From)]
#[reflect(Component)]
pub struct Rot(pub Quat);

impl Rot {
    #[cfg(feature = "2d")]
    pub fn rotate_vec3(&self, vec: Vec3) -> Vec3 {
        Vec3::new(
            vec.x * self.cos() - vec.y * self.sin(),
            vec.x * self.sin() + vec.y * self.cos(),
            vec.z,
        )
    }
    #[cfg(feature = "3d")]
    pub fn rotate_vec3(&self, vec: Vec3) -> Vec3 {
        self.0 * vec
    }
}

#[cfg(feature = "2d")]
impl Rot {
    pub const ZERO: Self = Self { cos: 1.0, sin: 0.0 };

    pub fn cos(&self) -> f32 {
        self.cos
    }

    pub fn sin(&self) -> f32 {
        self.sin
    }

    pub fn from_radians(radians: f32) -> Self {
        Self {
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    pub fn from_degrees(degrees: f32) -> Self {
        Self::from_radians(degrees.to_radians())
    }

    pub fn as_radians(&self) -> f32 {
        f32::atan2(self.sin(), self.cos())
    }

    pub fn as_degrees(&self) -> f32 {
        self.as_radians().to_degrees()
    }

    pub fn rotate(&self, vec: Vector) -> Vector {
        Vector::new(
            vec.x * self.cos() - vec.y * self.sin(),
            vec.x * self.sin() + vec.y * self.cos(),
        )
    }

    pub fn inv(&self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    pub fn mul(&self, rhs: Self) -> Self {
        Self {
            cos: self.cos * rhs.cos() - self.sin * rhs.sin(),
            sin: self.sin * rhs.cos() + self.cos * rhs.sin(),
        }
    }
}

#[cfg(feature = "3d")]
impl Rot {
    pub fn rotate(&self, vec: Vector) -> Vector {
        self.0 * vec
    }

    pub fn inv(&self) -> Self {
        Self(self.inverse())
    }
}

#[cfg(feature = "2d")]
impl Default for Rot {
    fn default() -> Self {
        Self::ZERO
    }
}

#[cfg(feature = "2d")]
impl Add<Self> for Rot {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        self.mul(rhs)
    }
}

#[cfg(feature = "3d")]
impl Add<Self> for Rot {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Rot(self.0 + rhs.0)
    }
}

impl AddAssign<Self> for Rot {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

#[cfg(feature = "2d")]
impl Sub<Self> for Rot {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        self.mul(rhs.inv())
    }
}

#[cfg(feature = "3d")]
impl Sub<Self> for Rot {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Rot(self.0 - rhs.0)
    }
}

impl SubAssign<Self> for Rot {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

#[cfg(feature = "2d")]
impl From<Rot> for f32 {
    fn from(rot: Rot) -> Self {
        rot.as_radians()
    }
}

#[cfg(feature = "2d")]
impl From<Rot> for Quat {
    fn from(rot: Rot) -> Self {
        if rot.cos() < 0.0 {
            let t = 1.0 - rot.cos();
            let d = 1.0 / (t * 2.0).sqrt();
            let z = -rot.sin() * d;
            let w = t * d;
            Quat::from_xyzw(0.0, 0.0, z, w)
        } else {
            let t = 1.0 + rot.cos();
            let d = 1.0 / (t * 2.0).sqrt();
            let z = t * d;
            let w = -rot.sin() * d;
            Quat::from_xyzw(0.0, 0.0, z, w)
        }
    }
}

#[cfg(feature = "3d")]
impl From<Rot> for Quat {
    fn from(rot: Rot) -> Self {
        rot.0
    }
}

#[cfg(feature = "3d")]
impl From<Rot> for Matrix3x1<f32> {
    fn from(rot: Rot) -> Self {
        Matrix3x1::new(rot.x, rot.y, rot.z)
    }
}

#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct PrevRot(pub Rot);
