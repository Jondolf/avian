use bevy::prelude::*;

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
/// The easiest way to build a [`CollisionLayers`] configuration is to use the [`CollisionLayers::new()`](#method.new) method
/// that takes in a list of groups and masks. Additional groups and masks can be added and removed by calling methods like
/// [`add_groups`](#method.add_groups), [`add_masks`](#method.add_masks), [`remove_groups`](#method.remove_groups) and
/// [`remove_masks`](#method.remove_masks).
///
/// These methods require the layers to implement [`PhysicsLayer`]. The easiest way to define the physics layers is to
/// create an enum with `#[derive(PhysicsLayer)]`.
///
/// Internally, the groups and masks are represented as bitmasks, so you can also use [`CollisionLayers::from_bits()`](#method.from_bits)
/// to create collision layers.
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
/// #[derive(PhysicsLayer)]
/// enum Layer {
///     Player,
///     Enemy,
///     Ground,
/// }
///
/// fn spawn(mut commands: Commands) {
///     commands.spawn((
///         Collider::ball(0.5),
///         // Player collides with enemies and the ground, but not with other players
///         CollisionLayers::new([Layer::Player], [Layer::Enemy, Layer::Ground])
///     ));
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[reflect(Component)]
pub struct CollisionLayers {
    groups: u32,
    masks: u32,
}

impl CollisionLayers {
    /// Creates a new [`CollisionLayers`] configuration with the given collision groups and masks.
    pub fn new<L: PhysicsLayer>(
        groups: impl IntoIterator<Item = L>,
        masks: impl IntoIterator<Item = L>,
    ) -> Self {
        Self::none().add_groups(groups).add_masks(masks)
    }

    /// Contains all groups and masks.
    pub fn all<L: PhysicsLayer>() -> Self {
        Self::from_bits(L::all_bits(), L::all_bits())
    }

    /// Contains all groups but no masks.
    pub fn all_groups<L: PhysicsLayer>() -> Self {
        Self::from_bits(L::all_bits(), 0)
    }

    /// Contains all masks but no groups.
    pub fn all_masks<L: PhysicsLayer>() -> Self {
        Self::from_bits(0, L::all_bits())
    }

    /// Contains no masks or groups.
    pub const fn none() -> Self {
        Self::from_bits(0, 0)
    }

    /// Creates a new [`CollisionLayers`] using bits.
    ///
    /// There is one bit per group and mask, so there are a total of 32 layers.
    /// For example, if an entity is a part of the layers `[0, 1, 3]` and can interact with the layers `[1, 2]`,
    /// the groups in bits would be `0b01011` while the masks would be `0b00110`.
    pub const fn from_bits(groups: u32, masks: u32) -> Self {
        Self { groups, masks }
    }

    /// Returns true if an entity with this [`CollisionLayers`] configuration
    /// can interact with an entity with the `other` [`CollisionLayers`] configuration.
    pub fn interacts_with(self, other: Self) -> bool {
        (self.groups & other.masks) != 0 && (other.groups & self.masks) != 0
    }

    /// Returns true if the given layer is contained in `groups`.
    pub fn contains_group(self, layer: impl PhysicsLayer) -> bool {
        (self.groups & layer.to_bits()) != 0
    }

    /// Adds the given layer into `groups`.
    pub fn add_group(mut self, layer: impl PhysicsLayer) -> Self {
        self.groups |= layer.to_bits();
        self
    }

    /// Adds the given layers into `groups`.
    pub fn add_groups(mut self, layers: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for layer in layers.into_iter().map(|l| l.to_bits()) {
            self.groups |= layer;
        }

        self
    }

    /// Removes the given layer from `groups`.
    pub fn remove_group(mut self, layer: impl PhysicsLayer) -> Self {
        self.groups &= !layer.to_bits();
        self
    }

    /// Removes the given layers from `groups`.
    pub fn remove_groups(mut self, layers: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for layer in layers.into_iter().map(|l| l.to_bits()) {
            self.groups &= !layer;
        }

        self
    }

    /// Returns true if the given layer is contained in `masks`.
    pub fn contains_mask(self, layer: impl PhysicsLayer) -> bool {
        (self.masks & layer.to_bits()) != 0
    }

    /// Adds the given layer into `masks`.
    pub fn add_mask(mut self, layer: impl PhysicsLayer) -> Self {
        self.masks |= layer.to_bits();
        self
    }

    /// Adds the given layers in `masks`.
    pub fn add_masks(mut self, layers: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for layer in layers.into_iter().map(|l| l.to_bits()) {
            self.masks |= layer;
        }

        self
    }

    /// Removes the given layer from `masks`.
    pub fn remove_mask(mut self, layer: impl PhysicsLayer) -> Self {
        self.masks &= !layer.to_bits();
        self
    }

    /// Removes the given layers from `masks`.
    pub fn remove_masks(mut self, layers: impl IntoIterator<Item = impl PhysicsLayer>) -> Self {
        for layer in layers.into_iter().map(|l| l.to_bits()) {
            self.masks &= !layer;
        }

        self
    }

    /// Returns the `groups` bitmask.
    pub fn groups_bits(self) -> u32 {
        self.groups
    }

    /// Returns the `masks` bitmask.
    pub fn masks_bits(self) -> u32 {
        self.masks
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        Self {
            groups: 0xffff_ffff,
            masks: 0xffff_ffff,
        }
    }
}
