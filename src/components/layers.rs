use std::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not};

use bevy::prelude::*;

/// A bitmask for layers.
#[derive(Reflect, Clone, Copy, Debug, Deref, DerefMut, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct LayerMask(pub u32);

impl From<u32> for LayerMask {
    fn from(layer: u32) -> Self {
        Self(layer)
    }
}

impl<L: PhysicsLayer> From<L> for LayerMask {
    fn from(layer: L) -> Self {
        LayerMask(layer.to_bits())
    }
}

impl<L: Into<LayerMask>, const N: usize> From<[L; N]> for LayerMask {
    fn from(value: [L; N]) -> Self {
        let mut bits = 0;

        for layer in value.into_iter().map(|l| {
            let layers: LayerMask = l.into();
            layers
        }) {
            bits |= layer.0;
        }

        LayerMask(bits)
    }
}

impl LayerMask {
    /// Contains all layers.
    pub const ALL: Self = Self(0xffff_ffff);
    /// Contains no layers.
    pub const NONE: Self = Self(0);
}

impl BitAnd for LayerMask {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitAndAssign for LayerMask {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 = self.0 & rhs.0;
    }
}

impl BitOr for LayerMask {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for LayerMask {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 = self.0 | rhs.0;
    }
}

impl BitXor for LayerMask {
    type Output = Self;

    fn bitxor(self, rhs: Self) -> Self::Output {
        Self(self.0 ^ rhs.0)
    }
}

impl BitXorAssign for LayerMask {
    fn bitxor_assign(&mut self, rhs: Self) {
        self.0 = self.0 ^ rhs.0;
    }
}

impl Not for LayerMask {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// A layer used for determining which entities should interact with each other.
/// Physics layers are used heavily by [`CollisionLayers`].
///
/// This trait can be derived for enums with `#[derive(PhysicsLayer)]`.
pub trait PhysicsLayer: Sized {
    /// Converts the layer to a bitmask.
    fn to_bits(&self) -> u32;
    /// Creates a layer bitmask with all bits set to 1.
    fn all_bits() -> u32;
}

impl<L: PhysicsLayer> PhysicsLayer for &L {
    fn to_bits(&self) -> u32 {
        L::to_bits(self)
    }

    fn all_bits() -> u32 {
        L::all_bits()
    }
}

/// Defines the collision layers of a collider using *groups* and *masks*.
///
/// **Groups** indicate what layers the collider is a part of.\
/// **Masks** indicate what layers the collider can interact with.
///
/// Two colliders `A` and `B` can interact if and only if:
///
/// - The groups of `A` contain a layer that is also in the masks of `B`
/// - The groups of `B` contain a layer that is also in the masks of `A`
///
/// Colliders without this component can be considered as having all groups and masks, and they can
/// interact with everything that belongs on any layer.
///
/// ## Creation
///
/// Collision layers store groups and masks using [`LayerMask`]s. A [`LayerMask`] can be created using
/// bitmasks, or by creating an enum that implements [`PhysicsLayer`].
///
/// Many [`CollisionLayers`] methods can take any type that implements `Into<Layers>`.
/// For example, you can use bitmasks with [`CollisionLayers::new`]:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// // Belongs to the second layer and interacts with colliders
/// // on the first, second, and third layer.
/// let layers = CollisionLayers::new(0b00010, 0b0111);
/// ```
///
/// You can also use an enum that implements [`PhysicsLayer`]:
///
/// ```
/// # use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// #[derive(PhysicsLayer)]
/// enum GameLayer {
///     Player,
///     Enemy,
///     Ground,
/// }
///
/// // Player collides with enemies and the ground, but not with other players
/// let layers = CollisionLayers::new(GameLayer::Player, [GameLayer::Enemy, GameLayer::Ground]);
/// ```
///
/// You can also use [`LayerMask`] directly:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use bevy_xpbd_3d::prelude::*;")]
/// #
/// // Belongs to the first layer and interacts with all layers.
/// let layers = CollisionLayers::new(LayerMask(0b0001), LayerMask::ALL);
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(Component)]
pub struct CollisionLayers {
    /// The layers that an entity belongs to.
    pub groups: LayerMask,
    /// The layers that an entity can interact with.
    pub masks: LayerMask,
}

impl CollisionLayers {
    /// Contains all groups and masks.
    pub const ALL: Self = Self {
        groups: LayerMask::ALL,
        masks: LayerMask::ALL,
    };

    /// Contains no groups or masks.
    pub const NONE: Self = Self {
        groups: LayerMask::NONE,
        masks: LayerMask::NONE,
    };

    /// Contains all groups but no masks.
    pub const ALL_GROUPS: Self = Self {
        groups: LayerMask::ALL,
        masks: LayerMask::NONE,
    };

    /// Contains all masks but no groups.
    pub const ALL_MASKS: Self = Self {
        groups: LayerMask::NONE,
        masks: LayerMask::ALL,
    };

    /// Creates a new [`CollisionLayers`] configuration with the given collision groups and masks.
    pub fn new(groups: impl Into<LayerMask>, masks: impl Into<LayerMask>) -> Self {
        Self {
            groups: groups.into(),
            masks: masks.into(),
        }
    }

    /// Creates a new [`CollisionLayers`] configuration using bits.
    ///
    /// There is one bit per group and mask, so there are a total of 32 layers.
    /// For example, if an entity is a part of the layers `[0, 1, 3]` and can interact with the layers `[1, 2]`,
    /// the groups in bits would be `0b01011` while the masks would be `0b00110`.
    pub const fn from_bits(groups: u32, masks: u32) -> Self {
        Self {
            groups: LayerMask(groups),
            masks: LayerMask(masks),
        }
    }

    /// Adds the given layers into `groups`.
    pub fn add_groups(&mut self, layers: impl Into<LayerMask>) {
        let layers: LayerMask = layers.into();
        self.groups |= layers;
    }

    /// Adds the given layers into `masks`.
    pub fn add_masks(&mut self, layers: impl Into<LayerMask>) {
        let layers: LayerMask = layers.into();
        self.masks |= layers;
    }

    /// Removes the given layers from `groups`.
    pub fn remove_groups(&mut self, layers: impl Into<LayerMask>) {
        let layers: LayerMask = layers.into();
        self.groups &= !layers;
    }

    /// Removes the given layers from `masks`.
    pub fn remove_masks(&mut self, layers: impl Into<LayerMask>) {
        let layers: LayerMask = layers.into();
        self.masks &= !layers;
    }

    /// Returns true if all of the given layers are contained in `groups`.
    pub fn contains_groups(self, layers: impl Into<LayerMask>) -> bool {
        let layers = layers.into();
        (self.groups & layers) != LayerMask::NONE
    }

    /// Returns true if all of the given layers are contained in `masks`.
    pub fn contains_masks(self, layers: impl Into<LayerMask>) -> bool {
        let layers = layers.into();
        (self.masks & layers) != LayerMask::NONE
    }

    /// Returns true if an entity with this [`CollisionLayers`] configuration
    /// can interact with an entity with the `other` [`CollisionLayers`] configuration.
    pub fn interacts_with(self, other: Self) -> bool {
        (self.groups & other.masks) != LayerMask::NONE
            && (other.groups & self.masks) != LayerMask::NONE
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        Self {
            groups: LayerMask::ALL,
            masks: LayerMask::ALL,
        }
    }
}

#[cfg(test)]
mod tests {
    // Needed for PhysicsLayer derive macro
    #[cfg(feature = "2d")]
    use crate as bevy_xpbd_2d;
    #[cfg(feature = "3d")]
    use crate as bevy_xpbd_3d;

    use crate::prelude::*;

    #[derive(PhysicsLayer)]
    enum GameLayer {
        Player,
        Enemy,
        Ground,
    }

    #[test]
    fn creation() {
        let with_bitmask = CollisionLayers::new(0b0010, 0b0101);
        let with_enum =
            CollisionLayers::new(GameLayer::Enemy, [GameLayer::Player, GameLayer::Ground]);
        let with_layers =
            CollisionLayers::new(LayerMask::from(GameLayer::Enemy), LayerMask(0b0101));

        assert_eq!(with_bitmask, with_enum);
        assert_eq!(with_bitmask, with_layers);

        assert!(with_bitmask.contains_groups(GameLayer::Enemy));
        assert!(!with_bitmask.contains_groups(GameLayer::Player));

        assert!(with_bitmask.contains_masks(GameLayer::Ground));
        assert!(!with_bitmask.contains_masks(GameLayer::Enemy));
    }
}
