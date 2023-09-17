use bevy::{prelude::*, sprite::MaterialMesh2dBundle, utils::HashSet};
use bevy_xpbd_2d::{math::*, prelude::*, PostProcessCollisionsSchedule};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(6))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, (movement, pass_through_one_way_platform))
        .add_systems(PostProcessCollisionsSchedule, one_way_platform)
        .run();
}

#[derive(Component)]
struct Actor;

#[derive(Clone, Eq, PartialEq, Debug, Default, Component)]
pub struct OneWayPlatform(HashSet<Entity>);

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default, Component, Reflect)]
pub enum PassThroughOneWayPlatform {
    #[default]
    /// Passes through a `OneWayPlatform` if the contact normal is in line with the platform's local-space up vector
    ByNormal,
    /// Always passes through a `OneWayPlatform`, temporarily set this to allow an actor to jump down through a platform
    Always,
    /// Never passes through a `OneWayPlatform`
    Never,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2dBundle::default());

    // For borders
    let square_sprite = Sprite {
        color: Color::rgb(0.7, 0.7, 0.8),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Ceiling
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::Y * 50.0 * 6.0),
        Collider::cuboid(50.0 * 20.0, 50.0),
    ));
    // Floor
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_Y * 50.0 * 6.0),
        Collider::cuboid(50.0 * 20.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::NEG_X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 11.0),
    ));
    // Right wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite,
            transform: Transform::from_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Position(Vector::X * 50.0 * 9.5),
        Collider::cuboid(50.0, 50.0 * 11.0),
    ));

    // For one-way platforms
    let one_way_sprite = Sprite {
        color: Color::rgba(0.7, 0.7, 0.8, 0.25),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Spawn some one way platforms
    for y in -2..=2 {
        commands.spawn((
            SpriteBundle {
                sprite: one_way_sprite.clone(),
                transform: Transform::from_scale(Vec3::new(10.0, 0.5, 1.0)),
                ..default()
            },
            RigidBody::Static,
            Position(Vector::Y * 15.0 * 6.0 * y as f32),
            Collider::cuboid(25.0 * 20.0, 25.0),
            OneWayPlatform::default(),
        ));
    }

    // Spawn an actor for the user to control
    let actor_size = Vec2::new(20.0, 20.0);
    let actor_mesh = MaterialMesh2dBundle {
        mesh: meshes.add(shape::Quad::new(actor_size).into()).into(),
        material: materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.9))),
        ..default()
    };

    commands.spawn((
        actor_mesh.clone(),
        RigidBody::Dynamic,
        LockedAxes::ROTATION_LOCKED,
        Position(Vector::ZERO),
        Collider::cuboid(actor_size.x, actor_size.y),
        Actor,
        PassThroughOneWayPlatform::ByNormal,
    ));
}

fn movement(
    keyboard_input: Res<Input<KeyCode>>,
    mut actors: Query<&mut LinearVelocity, With<Actor>>,
) {
    for mut linear_velocity in &mut actors {
        if keyboard_input.pressed(KeyCode::Left) {
            linear_velocity.x -= 10.0;
        }
        if keyboard_input.pressed(KeyCode::Right) {
            linear_velocity.x += 10.0;
        }

        if linear_velocity.y.abs() < 0.1 // Assume "mostly stopped" to mean "grounded"
            && !keyboard_input.pressed(KeyCode::Down)
            && keyboard_input.just_pressed(KeyCode::Space)
        {
            linear_velocity.y = 500.0;
        }
    }
}

fn pass_through_one_way_platform(
    keyboard_input: Res<Input<KeyCode>>,
    mut actors: Query<&mut PassThroughOneWayPlatform, With<Actor>>,
) {
    for mut pass_through_one_way_platform in &mut actors {
        if keyboard_input.pressed(KeyCode::Down) && keyboard_input.pressed(KeyCode::Space) {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::Always;
        } else {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::ByNormal;
        }
    }
}

/// Allows entities to pass through [`OneWayPlatform`] entities.
/// Passing through is achieved by removing the collisions between the [`OneWayPlatform`]
/// and the other entity if the entity should pass through.
/// If a [`PassThroughOneWayPlatform`] is present on the non-platform entity,
/// the value of the component dictates the pass-through behaviour.
///
/// Entities known to be passing through each [`OneWayPlatform`] are stored in the
/// [`OneWayPlatform`]. If an entity is known to be passing through a [`OneWayPlatform`],
/// it is allowed to continue to do so.
///
/// > Note that this is a very simplistic implementation of one-way
/// > platforms to demonstrate filtering collisions via [`PostProcessCollisionsSchedule`].
/// > You will probably want something more robust to implement one-way
/// > platforms properly, or may elect to use sensor colliders instead, which
/// > means you won't have collisions at all.
///
/// #### When an entity is known to already be passing through the [`OneWayPlatform`]
/// Any time an entity begins passing through a [`OneWayPlatform`], it is added to the
/// [`OneWayPlatform`]'s set of currently active penetrations, and will be allowed to
/// continue to pass through the platform until it is no longer penetrating the platform.
///
/// The entity is allowed to continue to pass through the platform as long as at least
/// one contact is penetrating.
///
/// Once all of the contacts are no longer penetrating the [`OneWayPlatform`], or all contacts
/// have stopped, the entity is forgotten about and the logic falls through to the next part.
///
/// #### When an entity is NOT known to be passing through the [`OneWayPlatform`]
/// Depending on the setting of [`PassThroughOneWayPlatform`], the entity may be allowed to
/// pass through.
///
/// If no [`PassThroughOneWayPlatform`] is present, [`PassThroughOneWayPlatform::ByNormal`] is used.
///
/// [`PassThroughOneWayPlatform`] may be in one of three states:
/// 1. [`PassThroughOneWayPlatform::ByNormal`]
///     - This is the default state
///     - The entity may be allowed to pass through the [`OneWayPlatform`] depending on the contact normal
///         - If all contact normals are in line with the [`OneWayPlatform`]'s local-space up vector,
///           the entity is allowed to pass through
/// 2. [`PassThroughOneWayPlatform::Always`]
///     - The entity will always pass through the [`OneWayPlatform`], regardless of contact normal
///     - This is useful for allowing an entity to jump down through a platform
/// 3. [`PassThroughOneWayPlatform::Never`]
///     - The entity will never pass through the [`OneWayPlatform`], meaning the platform will act
///       as normal hard collision for this entity
///
/// Even if an entity is changed to [`PassThroughOneWayPlatform::Never`], it will be allowed to pass
/// through a [`OneWayPlatform`] if it is already penetrating the platform. Once it exits the platform,
/// it will no longer be allowed to pass through.
fn one_way_platform(
    mut one_way_platforms_query: Query<(&mut OneWayPlatform, Option<&Rotation>)>,
    other_colliders_query: Query<
        Option<&PassThroughOneWayPlatform>,
        (With<Collider>, Without<OneWayPlatform>), // NOTE: This precludes OneWayPlatform passing through a OneWayPlatform
    >,
    rotations_query: Query<&Rotation>,
    mut collisions: ResMut<Collisions>,
) {
    // This assumes that Collisions contains empty entries for entities
    // that were once colliding but no longer are.

    collisions.retain(|(entity1, entity2), contacts| {
        // This is used in a couple of if statements below; writing here for brevity below.
        fn any_penetrating(contacts: &Contacts) -> bool {
            contacts.manifolds.iter().any(|manifold| {
                manifold
                    .contacts
                    .iter()
                    .any(|contact| contact.penetration > 0.)
            })
        }

        // First, figure out which entity is the one-way platform, and which is the other.
        // Choose the appropriate normal for pass-through depending on which is which.
        let (
            mut one_way_platform,
            platform_entity,
            other_entity,
            maybe_platform_rotation,
            pass_through_vector,
        ) = if let Ok((one_way_platform, maybe_platform_rotation)) =
            one_way_platforms_query.get_mut(*entity1)
        {
            (
                one_way_platform,
                entity1,
                entity2,
                maybe_platform_rotation,
                Vector::Y * 1.,
            )
        } else if let Ok((one_way_platform, maybe_platform_rotation)) =
            one_way_platforms_query.get_mut(*entity2)
        {
            (
                one_way_platform,
                entity2,
                entity1,
                maybe_platform_rotation,
                Vector::Y * -1.,
            )
        } else {
            // Neither is a one-way-platform, so accept the collision:
            // we're done here.
            return true;
        };

        if one_way_platform.0.contains(other_entity) {
            // If we were already allowing a collision for a particular entity,
            // and if it is penetrating us still, continue to allow it to do so.
            if any_penetrating(&contacts) {
                return false;
            } else {
                // If it's no longer penetrating us, forget it.
                one_way_platform.0.remove(other_entity);
            }
        }

        let pass_through_global_vector = maybe_platform_rotation
            .unwrap_or(&Rotation::default())
            .inverse()
            .rotate(pass_through_vector);

        match other_colliders_query.get(*other_entity) {
            // Pass-through is set to never, so accept the collision.
            Ok(Some(PassThroughOneWayPlatform::Never)) => true,
            // Pass-through is set to always, so always ignore this collision
            // and register it as an entity that's currently penetrating.
            Ok(Some(PassThroughOneWayPlatform::Always)) => {
                one_way_platform.0.insert(*other_entity);
                false
            }
            // Default behaviour is "by normal".
            Err(_) | Ok(None) | Ok(Some(PassThroughOneWayPlatform::ByNormal)) => {
                // Rotate the global pass through vector into the platform's local space.
                let pass_through_vector = rotations_query
                    .get(*platform_entity)
                    .unwrap_or(&Rotation::default())
                    .rotate(pass_through_global_vector);

                // If all contact normals are in line with the pass_through_vector of this platform
                // (in its local space), then ignore the collision and register it as an entity
                // that's currently penetrating.
                if contacts.manifolds.iter().all(|manifold| {
                    manifold.normal.length() > Scalar::EPSILON
                        && manifold.normal.dot(pass_through_vector) >= 0.5
                }) {
                    true
                } else if any_penetrating(&contacts) {
                    // If it's already penetrating, ignore the collision and register
                    // the other entity as one that's currently penetrating.
                    one_way_platform.0.insert(*other_entity);
                    false
                } else {
                    // In all other cases, allow this collision.
                    true
                }
            }
        }
    });
}
