//! **Contact queries** compute information about contacts between two [`Collider`]s.
//!
//! This module contains the following contact queries:
//!
//! | Contact query         | Description                                                    |
//! | --------------------- | -------------------------------------------------------------- |
//! | [`contact`]           | Computes one pair of contact points between two [`Collider`]s. |
//! | [`contact_manifolds`] | Computes all [`ContactManifold`]s between two [`Collider`]s.   |
//!
//! For geometric queries that query the entire world for intersections, like ray casting, shape casting
//! and point projection, see [spatial queries](spatial_query).

use crate::prelude::*;
use parry::query::PersistentQueryDispatcher;

/// Computes one pair of contact points between two [`Collider`]s.
///
/// Returns `None` if the colliders are separated by a distance greater than `prediction_distance`
/// or if the given shapes are invalid.
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
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// # {
/// let collider1 = Collider::ball(0.5);
/// let collider2 = Collider::cuboid(1.0, 1.0, 1.0);
///
/// // Compute a contact that should have a penetration depth of 0.5
/// let contact = contact(
///     // First collider
///     &collider1,
///     Vec3::default(),
///     Quat::default(),
///     // Second collider
///     &collider2,
///     Vec3::X * 0.5,
///     Quat::default(),
///     // Prediction distance
///     0.0,
/// );
///
/// assert_eq!(
///     contact.is_some_and(|contact| contact.penetration == 0.5),
///     true
/// );
/// # }
/// ```
pub fn contact(
    collider1: &Collider,
    position1: impl Into<Position>,
    rotation1: impl Into<Rotation>,
    collider2: &Collider,
    position2: impl Into<Position>,
    rotation2: impl Into<Rotation>,
    prediction_distance: Scalar,
) -> Option<ContactData> {
    let rotation1: Rotation = rotation1.into();
    let rotation2: Rotation = rotation2.into();
    let isometry1 = utils::make_isometry(position1.into(), rotation1);
    let isometry2 = utils::make_isometry(position2.into(), rotation2);

    if let Ok(Some(contact)) = parry::query::contact(
        &isometry1,
        collider1.get_shape().0.as_ref(),
        &isometry2,
        collider2.get_shape().0.as_ref(),
        prediction_distance,
    ) {
        // Transform contact data into local space
        let point1: Vector = rotation1.inverse().rotate(contact.point1.into());
        let point2: Vector = rotation2.inverse().rotate(contact.point2.into());
        let normal1: Vector = rotation1
            .inverse()
            .rotate(contact.normal1.into())
            .normalize();
        let normal2: Vector = rotation2
            .inverse()
            .rotate(contact.normal2.into())
            .normalize();

        // Make sure normals are valid
        if !normal1.is_normalized() || !normal2.is_normalized() {
            return None;
        }

        Some(ContactData {
            point1,
            point2,
            normal1,
            normal2,
            penetration: -contact.dist,
        })
    } else {
        None
    }
}

// Todo: Add a persistent version of this that tries to reuse previous contact manifolds
// by exploiting spatial and temporal coherence. This is supported by Parry's contact_manifolds,
// but requires using Parry's ContactManifold type.
/// Computes all [`ContactManifold`]s between two [`Collider`]s.
///
/// Returns an empty vector if the colliders are separated by a distance greater than `prediction_distance`
/// or if the given shapes are invalid.
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
/// # #[cfg(all(feature = "3d", feature = "f32"))]
/// # {
/// let collider1 = Collider::ball(0.5);
/// let collider2 = Collider::cuboid(1.0, 1.0, 1.0);
///
/// // Compute contact manifolds a collision that should be penetrating
/// let manifolds = contact_manifolds(
///     // First collider
///     &collider1,
///     Vec3::default(),
///     Quat::default(),
///     // Second collider
///     &collider2,
///     Vec3::X * 0.25,
///     Quat::default(),
///     // Prediction distance
///     0.0,
/// );
///
/// assert_eq!(manifolds.is_empty(), false);
/// # }
/// ```
pub fn contact_manifolds(
    collider1: &Collider,
    position1: impl Into<Position>,
    rotation1: impl Into<Rotation>,
    collider2: &Collider,
    position2: impl Into<Position>,
    rotation2: impl Into<Rotation>,
    prediction_distance: Scalar,
) -> Vec<ContactManifold> {
    let isometry1 = utils::make_isometry(position1.into(), rotation1.into());
    let isometry2 = utils::make_isometry(position2.into(), rotation2.into());
    let isometry12 = isometry1.inv_mul(&isometry2);

    // Todo: Reuse manifolds from previous frame to improve performance
    let mut manifolds: Vec<parry::query::ContactManifold<(), ()>> = vec![];
    let _ = parry::query::DefaultQueryDispatcher.contact_manifolds(
        &isometry12,
        collider1.get_shape().0.as_ref(),
        collider2.get_shape().0.as_ref(),
        prediction_distance,
        &mut manifolds,
        &mut None,
    );
    manifolds
        .iter()
        .filter_map(|manifold| {
            let subpos1 = manifold.subshape_pos1.unwrap_or_default();
            let subpos2 = manifold.subshape_pos2.unwrap_or_default();
            let normal1: Vector = subpos1
                .rotation
                .transform_vector(&manifold.local_n1)
                .normalize()
                .into();
            let normal2: Vector = subpos2
                .rotation
                .transform_vector(&manifold.local_n2)
                .normalize()
                .into();

            // Make sure normals are valid
            if !normal1.is_normalized() || !normal2.is_normalized() {
                return None;
            }

            Some(ContactManifold {
                normal1,
                normal2,
                contacts: manifold
                    .contacts()
                    .iter()
                    .map(|contact| ContactData {
                        point1: subpos1.transform_point(&contact.local_p1).into(),
                        point2: subpos2.transform_point(&contact.local_p2).into(),
                        normal1,
                        normal2,
                        penetration: -contact.dist,
                    })
                    .collect(),
            })
        })
        .collect()
}
