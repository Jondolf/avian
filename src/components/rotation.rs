use bevy::prelude::*;

pub trait Rotation: Default {
    const ZERO: Self;

    fn cos(&self) -> f32;

    fn sin(&self) -> f32;

    fn from_radians(radians: f32) -> Self;

    fn from_degrees(degrees: f32) -> Self {
        Self::from_radians(degrees.to_radians())
    }

    fn as_radians(&self) -> f32 {
        f32::atan2(self.sin(), self.cos())
    }

    fn as_degrees(&self) -> f32 {
        self.as_radians().to_degrees()
    }

    fn rotate(&self, vec: Vec2) -> Vec2 {
        Vec2::new(
            vec.x * self.cos() - vec.y * self.sin(),
            vec.x * self.sin() + vec.y * self.cos(),
        )
    }

    fn inv(&self) -> Self;

    fn mul<T: Rotation>(&self, rhs: T) -> Self;
}

#[derive(Clone, Copy, Component, Debug)]
pub struct Rot {
    pub cos: f32,
    pub sin: f32,
}

impl Rotation for Rot {
    const ZERO: Self = Self { cos: 1.0, sin: 0.0 };

    fn cos(&self) -> f32 {
        self.cos
    }

    fn sin(&self) -> f32 {
        self.sin
    }

    fn from_radians(radians: f32) -> Self {
        Self {
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    fn inv(&self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    fn mul<T: Rotation>(&self, rhs: T) -> Self {
        Self {
            cos: self.cos * rhs.cos() - self.sin * rhs.sin(),
            sin: self.sin * rhs.cos() + self.cos * rhs.sin(),
        }
    }
}

impl Default for Rot {
    fn default() -> Self {
        Self::ZERO
    }
}

#[derive(Clone, Copy, Component, Debug)]
pub struct PrevRot {
    pub cos: f32,
    pub sin: f32,
}

impl Rotation for PrevRot {
    const ZERO: Self = Self { cos: 1.0, sin: 0.0 };

    fn cos(&self) -> f32 {
        self.cos
    }

    fn sin(&self) -> f32 {
        self.sin
    }

    fn from_radians(radians: f32) -> Self {
        Self {
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    fn inv(&self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    fn mul<T: Rotation>(&self, rhs: T) -> Self {
        Self {
            cos: self.cos * rhs.cos() - self.sin * rhs.sin(),
            sin: self.sin * rhs.cos() + self.cos * rhs.sin(),
        }
    }
}

fn quat_from_rot<T: Rotation>(rot: T) -> Quat {
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

impl Default for PrevRot {
    fn default() -> Self {
        Self::ZERO
    }
}

impl From<Rot> for Quat {
    fn from(rot: Rot) -> Self {
        quat_from_rot(rot)
    }
}

impl From<PrevRot> for Quat {
    fn from(rot: PrevRot) -> Self {
        quat_from_rot(rot)
    }
}
