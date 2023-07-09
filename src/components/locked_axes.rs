use bevy::prelude::*;
use derive_more::From;

use crate::prelude::*;

/// A component that specifies which translational and rotational axes of a [rigid body](RigidBody) are locked.
///
/// The axes are represented using a total of six bits, one for each axis. The easiest way is to lock or unlock
/// specific axes is to use the methods like [`lock_translation_x`](#method.lock_translation_x), but you can also
/// use bits directly with the [`from_bits`](#method.from_bits) and [`to_bits`](#method.to_bits) methods.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
/// # #[cfg(feature = "2d")]
/// # use bevy_xpbd_2d::prelude::*;
/// # #[cfg(feature = "3d")]
/// use bevy_xpbd_3d::prelude::*;
///
/// #[cfg(feature = "3d")]
/// fn spawn(mut commands: Commands) {
///     // Spawn a capsule that only rotates around the Y axis
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(1.0, 0.5),
///         // In 2D, use LockedAxes::new().lock_rotation()
///         LockedAxes::new().lock_rotation_x().lock_rotation_z(),
///     ));
/// }
/// ```
#[derive(Component, Reflect, Clone, Copy, Debug, Default, From)]
#[reflect(Component)]
pub struct LockedAxes(u8);

impl LockedAxes {
    /// All translational axes are locked, but all rotational axes are unlocked.
    pub const TRANSLATION_LOCKED: Self = Self(0b111_000);
    /// All rotational axes are locked, but all translational axes are unlocked.
    pub const ROTATION_LOCKED: Self = Self(0b000_111);
    /// All translational and rotational axes are locked.
    pub const ALL_LOCKED: Self = Self(0b111_111);

    /// Creates a new [`LockedAxes`] configuration with all axes unlocked by default.
    pub fn new() -> Self {
        Self(0)
    }

    /// Creates a new [`LockedAxes`] configuration using bits.
    ///
    /// The first three bits correspond to translational axes, while the last three bits correspond to rotational
    /// axes. For example, `0b100_010` would lock translation along the `X` axis and rotation around the `Y` axis.
    pub fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Returns the locked axes as bits.
    ///
    /// The first three bits correspond to translational axes, while the last three bits correspond to rotational
    /// axes. For example, `0b100_010` would mean that translation along the `X` axis and rotation around the `Y` axis
    /// are locked.
    pub fn to_bits(&self) -> u8 {
        self.0
    }

    /// Locks translation along the `X` axis.
    pub fn lock_translation_x(mut self) -> Self {
        self.0 |= 0b100_000;
        self
    }

    /// Locks translation along the `Y` axis.
    pub fn lock_translation_y(mut self) -> Self {
        self.0 |= 0b010_000;
        self
    }

    /// Locks translation along the `Z` axis.
    #[cfg(feature = "3d")]
    pub fn lock_translation_z(mut self) -> Self {
        self.0 |= 0b001_000;
        self
    }

    /// Locks rotation around the `X` axis.
    #[cfg(feature = "3d")]
    pub fn lock_rotation_x(mut self) -> Self {
        self.0 |= 0b000_100;
        self
    }

    /// Locks rotation around the `Y` axis.
    #[cfg(feature = "3d")]
    pub fn lock_rotation_y(mut self) -> Self {
        self.0 |= 0b000_010;
        self
    }

    /// Locks rotation around the `Z` axis.
    #[cfg(feature = "3d")]
    pub fn lock_rotation_z(mut self) -> Self {
        self.0 |= 0b000_001;
        self
    }

    /// Locks all rotation.
    #[cfg(feature = "2d")]
    pub fn lock_rotation(mut self) -> Self {
        self.0 |= 0b000_001;
        self
    }

    /// Unlocks translation along the `X` axis.
    pub fn unlock_translation_x(mut self) -> Self {
        self.0 &= !0b100_000;
        self
    }

    /// Unlocks translation along the `Y` axis.
    pub fn unlock_translation_y(mut self) -> Self {
        self.0 &= !0b010_000;
        self
    }

    /// Unlocks translation along the `Z` axis.
    #[cfg(feature = "3d")]
    pub fn unlock_translation_z(mut self) -> Self {
        self.0 &= !0b001_000;
        self
    }

    /// Unlocks rotation around the `X` axis.
    #[cfg(feature = "3d")]
    pub fn unlock_rotation_x(mut self) -> Self {
        self.0 &= !0b000_100;
        self
    }

    /// Unlocks rotation around the `Y` axis.
    #[cfg(feature = "3d")]
    pub fn unlock_rotation_y(mut self) -> Self {
        self.0 &= !0b000_010;
        self
    }

    /// Unlocks rotation around the `Z` axis.
    #[cfg(feature = "3d")]
    pub fn unlock_rotation_z(mut self) -> Self {
        self.0 &= !0b000_001;
        self
    }

    /// Unlocks all rotation.
    #[cfg(feature = "2d")]
    pub fn unlock_rotation(mut self) -> Self {
        self.0 &= !0b000_001;
        self
    }

    /// Returns true if translation is locked along the `X` axis.
    pub fn is_translation_x_locked(&self) -> bool {
        (self.0 & 0b100_000) != 0
    }

    /// Returns true if translation is locked along the `X` axis.
    pub fn is_translation_y_locked(&self) -> bool {
        (self.0 & 0b010_000) != 0
    }

    /// Returns true if translation is locked along the `X` axis.
    #[cfg(feature = "3d")]
    pub fn is_translation_z_locked(&self) -> bool {
        (self.0 & 0b001_000) != 0
    }

    /// Returns true if rotation is locked around the `X` axis.
    #[cfg(feature = "3d")]
    pub fn is_rotation_x_locked(&self) -> bool {
        (self.0 & 0b000_100) != 0
    }

    /// Returns true if rotation is locked around the `Y` axis.
    #[cfg(feature = "3d")]
    pub fn is_rotation_y_locked(&self) -> bool {
        (self.0 & 0b000_010) != 0
    }

    /// Returns true if rotation is locked around the `Z` axis.
    #[cfg(feature = "3d")]
    pub fn is_rotation_z_locked(&self) -> bool {
        (self.0 & 0b000_001) != 0
    }

    /// Returns true if all rotation is locked.
    #[cfg(feature = "2d")]
    pub fn is_rotation_locked(&self) -> bool {
        (self.0 & 0b000_001) != 0
    }

    /// Sets translational axes of the given vector to zero based on the [`LockedAxes`] configuration.
    pub(crate) fn apply_to_vec(&self, mut vector: Vector) -> Vector {
        if self.is_translation_x_locked() {
            vector.x = 0.0;
        }
        if self.is_translation_y_locked() {
            vector.y = 0.0;
        }
        #[cfg(feature = "3d")]
        if self.is_translation_z_locked() {
            vector.z = 0.0;
        }
        vector
    }

    /// Sets the given rotation to zero if rotational axes are locked.
    #[cfg(feature = "2d")]
    pub(crate) fn apply_to_rotation(&self, mut rotation: Scalar) -> Scalar {
        if self.is_rotation_locked() {
            rotation = 0.0;
        }
        rotation
    }

    /// Sets rotational axes of the given 3x3 matrix to zero based on the [`LockedAxes`] configuration.
    #[cfg(feature = "3d")]
    pub(crate) fn apply_to_rotation(&self, mut rotation: Matrix3) -> Matrix3 {
        if self.is_rotation_x_locked() {
            rotation.x_axis = Vector::ZERO;
        }
        if self.is_rotation_y_locked() {
            rotation.y_axis = Vector::ZERO;
        }
        if self.is_rotation_z_locked() {
            rotation.z_axis = Vector::ZERO;
        }
        rotation
    }
}
