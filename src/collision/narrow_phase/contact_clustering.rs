use crate::prelude::*;

/// Merges manifolds with nearly parallel normals and prunes each cluster to at most 4 points.
pub(super) fn cluster_manifolds(
    manifolds: &mut Vec<ContactManifold>,
    angle_cos: f32,
    distance_threshold: f32,
) {
    let mut cluster_count = 0;

    // Merge manifolds with nearly parallel normals.
    for i in 0..manifolds.len() {
        if manifolds[i].points.is_empty() {
            continue;
        }

        let normal = manifolds[i].normal;
        if let Some(cluster_index) = manifolds[..cluster_count]
            .iter()
            .position(|cluster| cluster.normal.dot(normal) >= angle_cos)
        {
            let mut points = core::mem::take(&mut manifolds[i].points);
            manifolds[cluster_index].points.append(&mut points);
        } else {
            manifolds.swap(cluster_count, i);
            cluster_count += 1;
        }
    }
    manifolds.truncate(cluster_count);

    // Deduplicate points within each cluster and prune to at most 4 points.
    for manifold in manifolds {
        deduplicate_points(&mut manifold.points, distance_threshold);
        if manifold.points.len() > 4 {
            manifold.prune_points();
        }
    }
}

/// Removes duplicate contacts, keeping the deepest point in each group.
pub(super) fn deduplicate_points(points: &mut Vec<ContactPoint>, distance_threshold: f32) {
    let distance_squared = distance_threshold.powi(2);
    let mut point_count = 0;

    for i in 0..points.len() {
        let point = points[i];
        if let Some(existing) = points[..point_count].iter_mut().find(|existing| {
            existing.anchor1.distance_squared(point.anchor1) <= distance_squared
                && existing.anchor2.distance_squared(point.anchor2) <= distance_squared
        }) {
            if point.penetration > existing.penetration {
                *existing = point;
            }
        } else {
            points[point_count] = point;
            point_count += 1;
        }
    }
    points.truncate(point_count);
}

/// Matches cluster points by position.
pub(super) fn match_contact_cluster(
    manifold: &mut ContactManifold,
    previous_manifolds: &mut [ContactManifold],
    angle_cos: f32,
    distance_threshold: f32,
) {
    let distance_threshold_squared = distance_threshold.powi(2);

    for point in &mut manifold.points {
        let mut closest = None;
        let mut closest_distance = f32::MAX;

        for (manifold_index, previous_manifold) in previous_manifolds.iter().enumerate() {
            if manifold.normal.dot(previous_manifold.normal) < angle_cos {
                continue;
            }

            for (point_index, previous_point) in previous_manifold.points.iter().enumerate() {
                let distance1 = point.anchor1.distance_squared(previous_point.anchor1);
                let distance2 = point.anchor2.distance_squared(previous_point.anchor2);
                let distance = distance1 + distance2;
                if distance1 <= distance_threshold_squared
                    && distance2 <= distance_threshold_squared
                    && distance < closest_distance
                {
                    closest = Some((manifold_index, point_index));
                    closest_distance = distance;
                }
            }
        }

        if let Some((manifold_index, point_index)) = closest {
            let previous_point = previous_manifolds[manifold_index]
                .points
                .swap_remove(point_index);
            point.warm_start_normal_impulse = previous_point.warm_start_normal_impulse;
            point.warm_start_tangent_impulse = previous_point.warm_start_tangent_impulse;
        }
    }
}
