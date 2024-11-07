use crate::prelude::*;
use bevy::prelude::*;

/// Determines how coefficients are combined for [`Restitution`] and [`Friction`].
/// The default is `Average`.
///
/// When combine rules clash with each other, the following priority order is used:
/// `Max > Multiply > Min > GeometricMean > Average`.
#[derive(Reflect, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub enum CoefficientCombine {
    /// Coefficients are combined by computing their average `(a + b) / 2.0`.
    Average = 1,
    /// Coefficients are combined by computing their geometric mean `sqrt(a * b)`.
    GeometricMean = 2,
    /// Coefficients are combined by choosing the smaller coefficient `min(a, b)`.
    Min = 3,
    /// Coefficients are combined by computing their product `a * b`.
    Multiply = 4,
    /// Coefficients are combined by choosing the larger coefficient `max(a, b)`.
    Max = 5,
}

impl CoefficientCombine {
    /// Combines two coefficients according to the combine rule.
    pub fn mix(&self, a: Scalar, b: Scalar) -> Scalar {
        match self {
            CoefficientCombine::Average => (a + b) * 0.5,
            CoefficientCombine::GeometricMean => (a * b).sqrt(),
            CoefficientCombine::Min => a.min(b),
            CoefficientCombine::Multiply => a * b,
            CoefficientCombine::Max => a.max(b),
        }
    }
}

/// A resource for the default [`Friction`] to use for physics objects.
///
/// Friction can be set for individual colliders and rigid bodies using the [`Friction`] component.
///
/// Defaults to dynamic and static friction coefficients of `0.6` with a mixing rule of [`CoefficientCombine::Average`].
/// `0.6` is close to common real-world friction for most projects, resembling a material similar to concrete or brick.
#[derive(Resource, Clone, Copy, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct DefaultFriction(pub Friction);

/// A resource for the default [`Restitution`] to use for physics objects.
///
/// Restitution can be set for individual colliders and rigid bodies using the [`Restitution`] component.
///
/// Defaults to a coefficient of `0.0` with a mixing rule of [`CoefficientCombine::Average`].
#[derive(Resource, Clone, Copy, Debug, Default, Deref, DerefMut, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct DefaultRestitution(pub Restitution);

/// A component for the [coefficient of restitution], controlling how bouncy a [rigid body](RigidBody) or [collider](Collider) is.
///
/// The coefficient should be between 0 and 1, where 0 corresponds to a **perfectly inelastic** collision with zero bounce,
/// and 1 corresponds to a **perfectly elastic** collision that tries to preserve all kinetic energy. Note that some kinetic energy
/// can still be lost over long periods of time due to simulation inaccuracies or friction, even with a coefficient of 1.
///
/// When two bodies collide, their restitution coefficients are combined using the specified [`CoefficientCombine`] rule.
///
/// Entities without a [`Restitution`] component use the [`DefaultRestitution`] resource for collisions.
/// By default, restitution is set to zero, and coefficients are averaged according to [`CoefficientCombine::Average`].
///
/// [coefficient of restitution]: https://en.wikipedia.org/wiki/Coefficient_of_restitution
///
/// # Example
///
/// Create a new [`Restitution`] component with a restitution coefficient of `0.4`:
///
/// ```ignore
/// Restitution::new(0.4)
/// ```
///
/// Configure how two restitution coefficients are combined with [`CoefficientCombine`]:
///
/// ```ignore
/// Restitution::new(0.4).with_combine_rule(CoefficientCombine::Max)
/// ```
///
/// Combine the properties of two [`Restitution`] components:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
///
/// let first = Restitution::new(0.8).with_combine_rule(CoefficientCombine::Average);
/// let second = Restitution::new(0.5).with_combine_rule(CoefficientCombine::Multiply);
///
/// // `CoefficientCombine::Multiply` has higher priority, so the coefficients are multiplied
/// assert_eq!(
///     first.combine(second),
///     Restitution::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// );
/// ```
#[doc(alias = "Bounciness")]
#[doc(alias = "Elasticity")]
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct Restitution {
    /// The [coefficient of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
    ///
    /// This should be between 0 and 1, where 0 corresponds to a **perfectly inelastic** collision with zero bounce,
    /// and 1 corresponds to a **perfectly elastic** collision that tries to preserve all kinetic energy. Note that some kinetic energy
    /// can still be lost over long periods of time due to simulation inaccuracies or friction, even with a coefficient of 1.
    ///
    /// Defaults to `0.0`.
    pub coefficient: Scalar,
    /// The coefficient combine rule used when two bodies collide.
    ///
    /// Defaults to [`CoefficientCombine::Average`].
    pub combine_rule: CoefficientCombine,
}

impl Default for Restitution {
    /// The default [`Restitution`] with a coefficient of `0.0` and a combine rule of [`CoefficientCombine::Average`].
    fn default() -> Self {
        Self {
            coefficient: 0.0,
            combine_rule: CoefficientCombine::Average,
        }
    }
}

impl Restitution {
    /// A restitution coefficient of `0.0` and a combine rule of [`CoefficientCombine::Average`].
    ///
    /// This is equivalent to [`Restitution::PERFECTLY_INELASTIC`].
    pub const ZERO: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// A restitution coefficient of `0.0`, which corresponds to a perfectly inelastic collision.
    ///
    /// Uses [`CoefficientCombine::Average`].
    pub const PERFECTLY_INELASTIC: Self = Self {
        coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// A restitution coefficient of `1.0`, which corresponds to a perfectly elastic collision.
    ///
    /// Uses [`CoefficientCombine::Average`].
    pub const PERFECTLY_ELASTIC: Self = Self {
        coefficient: 1.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new [`Restitution`] component with the given restitution coefficient.
    pub fn new(coefficient: Scalar) -> Self {
        Self {
            coefficient: coefficient.clamp(0.0, 1.0),
            combine_rule: CoefficientCombine::Average,
        }
    }

    /// Sets the [`CoefficientCombine`] rule used.
    pub fn with_combine_rule(&self, combine_rule: CoefficientCombine) -> Self {
        Self {
            combine_rule,
            ..*self
        }
    }

    /// Combines the properties of two [`Restitution`] components.
    pub fn combine(&self, other: Self) -> Self {
        // Choose rule with higher priority
        let rule = self.combine_rule.max(other.combine_rule);

        Self {
            coefficient: rule.mix(self.coefficient, other.coefficient),
            combine_rule: rule,
        }
    }
}

impl From<Scalar> for Restitution {
    fn from(coefficient: Scalar) -> Self {
        Self {
            coefficient,
            ..default()
        }
    }
}

/// Controls how strongly the material of an entity prevents sliding along surfaces.
///
/// For surfaces that are at rest relative to each other, **static friction** is used.
/// Once the static friction is overcome, the bodies will start sliding relative to each other, and **dynamic friction** is applied instead.
///
/// `0.0`: No friction at all, the body slides indefinitely\
/// `1.0`: High friction
///
/// Entities without a [`Friction`] component use the [`DefaultFriction`] resource for collisions.
/// By default, dynamic and static friction coefficients are set to `0.6`, and coefficients are averaged according to [`CoefficientCombine::Average`].
/// `0.6` is close to common real-world friction for most projects, resembling a material similar to concrete or brick.
///
/// # Example
///
/// Create a new [`Friction`] component with dynamic and static friction coefficients of 0.4:
///
/// ```ignore
/// Friction::new(0.4)
/// ```
///
/// Set the other friction coefficient:
///
/// ```ignore
/// // 0.4 static and 0.6 dynamic
/// Friction::new(0.4).with_dynamic_coefficient(0.6)
/// // 0.4 dynamic and 0.6 static
/// Friction::new(0.4).with_static_coefficient(0.6)
/// ```
///
/// Configure how the friction coefficients of two [`Friction`] components are combined with [`CoefficientCombine`]:
///
/// ```ignore
/// Friction::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// ```
///
/// Combine the properties of two [`Friction`] components:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
///
/// let first = Friction::new(0.8).with_combine_rule(CoefficientCombine::Average);
/// let second = Friction::new(0.5).with_combine_rule(CoefficientCombine::Multiply);
///
/// // CoefficientCombine::Multiply has higher priority, so the coefficients are multiplied
/// assert_eq!(
///     first.combine(second),
///     Friction::new(0.4).with_combine_rule(CoefficientCombine::Multiply)
/// );
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct Friction {
    /// Coefficient of dynamic friction.
    ///
    /// Defaults to `0.6`.
    pub dynamic_coefficient: Scalar,
    /// Coefficient of static friction.
    ///
    /// Defaults to `0.6`.
    pub static_coefficient: Scalar,
    /// The coefficient combine rule used when two bodies collide.
    ///
    /// Defaults to [`CoefficientCombine::Average`].
    pub combine_rule: CoefficientCombine,
}

// TODO: Use geometric mean for default combine rule
impl Default for Friction {
    /// The default [`Friction`] with dynamic and static friction coefficients of `0.6` and a combine rule of [`CoefficientCombine::Average`].
    fn default() -> Self {
        Self {
            dynamic_coefficient: 0.6,
            static_coefficient: 0.6,
            combine_rule: CoefficientCombine::Average,
        }
    }
}

impl Friction {
    /// Zero dynamic and static friction and [`CoefficientCombine::Average`].
    pub const ZERO: Self = Self {
        dynamic_coefficient: 0.0,
        static_coefficient: 0.0,
        combine_rule: CoefficientCombine::Average,
    };

    /// Creates a new `Friction` component with the same dynamic and static friction coefficients.
    pub fn new(friction_coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: friction_coefficient,
            static_coefficient: friction_coefficient,
            ..default()
        }
    }

    /// Sets the [`CoefficientCombine`] rule used.
    pub fn with_combine_rule(&self, combine_rule: CoefficientCombine) -> Self {
        Self {
            combine_rule,
            ..*self
        }
    }

    /// Sets the coefficient of dynamic friction.
    pub fn with_dynamic_coefficient(&self, coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: coefficient,
            ..*self
        }
    }

    /// Sets the coefficient of static friction.
    pub fn with_static_coefficient(&self, coefficient: Scalar) -> Self {
        Self {
            static_coefficient: coefficient,
            ..*self
        }
    }

    /// Combines the properties of two `Friction` components.
    pub fn combine(&self, other: Self) -> Self {
        // Choose rule with higher priority
        let rule = self.combine_rule.max(other.combine_rule);

        Self {
            dynamic_coefficient: rule.mix(self.dynamic_coefficient, other.dynamic_coefficient),
            static_coefficient: rule.mix(self.static_coefficient, other.static_coefficient),
            combine_rule: rule,
        }
    }
}

impl From<Scalar> for Friction {
    fn from(coefficient: Scalar) -> Self {
        Self {
            dynamic_coefficient: coefficient,
            static_coefficient: coefficient,
            ..default()
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;
    use approx::assert_relative_eq;

    #[test]
    fn restitution_clamping_works() {
        assert_eq!(Restitution::new(-2.0).coefficient, 0.0);
        assert_eq!(Restitution::new(0.6).coefficient, 0.6);
        assert_eq!(Restitution::new(3.0).coefficient, 1.0);
    }

    // TODO: Test `CoefficientCombine` directly
    // TODO: Test geometric mean
    #[test]
    fn coefficient_combine_works() {
        let r1 = Restitution::new(0.3).with_combine_rule(CoefficientCombine::Average);

        // (0.3 + 0.7) / 2.0 == 0.5
        let average_result =
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Average));
        let average_expected = Restitution::new(0.5).with_combine_rule(CoefficientCombine::Average);
        assert_relative_eq!(
            average_result.coefficient,
            average_expected.coefficient,
            epsilon = 0.0001
        );
        assert_eq!(average_result.combine_rule, average_expected.combine_rule);

        // 0.3.min(0.7) == 0.3
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Min)),
            Restitution::new(0.3).with_combine_rule(CoefficientCombine::Min)
        );

        // 0.3 * 0.7 == 0.21
        let multiply_result =
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Multiply));
        let multiply_expected =
            Restitution::new(0.21).with_combine_rule(CoefficientCombine::Multiply);
        assert_relative_eq!(
            multiply_result.coefficient,
            multiply_expected.coefficient,
            epsilon = 0.0001
        );
        assert_eq!(multiply_result.combine_rule, multiply_expected.combine_rule);

        // 0.3.max(0.7) == 0.7
        assert_eq!(
            r1.combine(Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)),
            Restitution::new(0.7).with_combine_rule(CoefficientCombine::Max)
        );
    }
}
