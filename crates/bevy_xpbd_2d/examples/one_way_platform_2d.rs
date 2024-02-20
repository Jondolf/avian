//! A 2D platformer example with one-way platforms to demonstrate
//! filtering collisions with systems in the `PostProcessCollisions` schedule.
//!
//! Move with arrow keys, jump with Space and descend through
//! platforms by pressing Space while holding the down arrow.

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, utils::HashSet};
use bevy_xpbd_2d::{math::*, prelude::*, PostProcessCollisions};
use examples_common_2d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(6))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, (movement, pass_through_one_way_platform))
        .add_systems(PostProcessCollisions, one_way_platform)
        .run();
}

#[derive(Component)]
struct Actor;

#[derive(Component)]
struct MovementSpeed(Scalar);

#[derive(Component)]
struct JumpImpulse(Scalar);

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
            transform: Transform::from_xyz(0.0, 50.0 * 6.0, 0.0)
                .with_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Floor
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_xyz(0.0, -50.0 * 6.0, 0.0)
                .with_scale(Vec3::new(20.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite.clone(),
            transform: Transform::from_xyz(-50.0 * 9.5, 0.0, 0.0)
                .with_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Right wall
    commands.spawn((
        SpriteBundle {
            sprite: square_sprite,
            transform: Transform::from_xyz(50.0 * 9.5, 0.0, 0.0)
                .with_scale(Vec3::new(1.0, 11.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
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
                transform: Transform::from_xyz(0.0, y as f32 * 16.0 * 6.0, 0.0)
                    .with_scale(Vec3::new(10.0, 0.5, 1.0)),
                ..default()
            },
            RigidBody::Static,
            Collider::rectangle(50.0, 50.0),
            OneWayPlatform::default(),
        ));
    }

    // Spawn an actor for the user to control
    let actor_size = Vector::new(20.0, 20.0);
    let actor_mesh = MaterialMesh2dBundle {
        mesh: meshes.add(Rectangle::from_size(actor_size.f32())).into(),
        material: materials.add(Color::rgb(0.2, 0.7, 0.9)),
        ..default()
    };

    commands.spawn((
        actor_mesh.clone(),
        RigidBody::Dynamic,
        LockedAxes::ROTATION_LOCKED,
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        Collider::rectangle(actor_size.x, actor_size.y),
        Actor,
        PassThroughOneWayPlatform::ByNormal,
        MovementSpeed(250.0),
        JumpImpulse(450.0),
    ));
}

fn movement(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut actors: Query<(&mut LinearVelocity, &MovementSpeed, &JumpImpulse), With<Actor>>,
) {
    for (mut linear_velocity, movement_speed, jump_impulse) in &mut actors {
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);
        let horizontal = right as i8 - left as i8;

        // Move in input direction
        linear_velocity.x = horizontal as Scalar * movement_speed.0;

        // Assume "mostly stopped" to mean "grounded".
        // You should use raycasting, shapecasting or sensor colliders
        // for more robust ground detection.
        if linear_velocity.y.abs() < 0.1
            && !keyboard_input.pressed(KeyCode::ArrowDown)
            && keyboard_input.just_pressed(KeyCode::Space)
        {
            linear_velocity.y = jump_impulse.0;
        }
    }
}

fn pass_through_one_way_platform(
    mut commands: Commands,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut actors: Query<(Entity, &mut PassThroughOneWayPlatform), With<Actor>>,
) {
    for (entity, mut pass_through_one_way_platform) in &mut actors {
        if keyboard_input.pressed(KeyCode::ArrowDown) && keyboard_input.pressed(KeyCode::Space) {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::Always;

            // Wake up body when it's allowed to drop down.
            // Otherwise it won't fall because gravity isn't simulated.
            commands.entity(entity).remove::<Sleeping>();
        } else {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::ByNormal;
        }
    }
}

/// Allows entities to pass through [`OneWayPlatform`] entities.
///
/// Passing through is achieved by removing the collisions between the [`OneWayPlatform`]
/// and the other entity if the entity should pass through.
/// If a [`PassThroughOneWayPlatform`] is present on the non-platform entity,
/// the value of the component dictates the pass-through behaviour.
///
/// Entities known to be passing through each [`OneWayPlatform`] are stored in the
/// [`OneWayPlatform`]. If an entity is known to be passing through a [`OneWayPlatform`],
/// it is allowed to continue to do so, even if [`PassThroughOneWayPlatform`] has been
/// set to disallow passing through.
///
/// > Note that this is a very simplistic implementation of one-way
/// > platforms to demonstrate filtering collisions via [`PostProcessCollisions`].
/// > You will probably want something more robust to implement one-way
/// > platforms properly, or may elect to use a sensor collider for your entities instead,
/// > which means you won't need to filter collisions at all.
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
    mut one_way_platforms_query: Query<&mut OneWayPlatform>,
    other_colliders_query: Query<
        Option<&PassThroughOneWayPlatform>,
        (With<Collider>, Without<OneWayPlatform>), // NOTE: This precludes OneWayPlatform passing through a OneWayPlatform
    >,
    mut collisions: ResMut<Collisions>,
) {
    // This assumes that Collisions contains empty entries for entities
    // that were once colliding but no longer are.
    collisions.retain(|contacts| {
        // This is used in a couple of if statements below; writing here for brevity below.
        fn any_penetrating(contacts: &Contacts) -> bool {
            contacts.manifolds.iter().any(|manifold| {
                manifold
                    .contacts
                    .iter()
                    .any(|contact| contact.penetration > 0.0)
            })
        }

        // Differentiate between which normal of the manifold we should use
        enum RelevantNormal {
            Normal1,
            Normal2,
        }

        // First, figure out which entity is the one-way platform, and which is the other.
        // Choose the appropriate normal for pass-through depending on which is which.
        let (mut one_way_platform, other_entity, relevant_normal) =
            if let Ok(one_way_platform) = one_way_platforms_query.get_mut(contacts.entity1) {
                (one_way_platform, contacts.entity2, RelevantNormal::Normal1)
            } else if let Ok(one_way_platform) = one_way_platforms_query.get_mut(contacts.entity2) {
                (one_way_platform, contacts.entity1, RelevantNormal::Normal2)
            } else {
                // Neither is a one-way-platform, so accept the collision:
                // we're done here.
                return true;
            };

        if one_way_platform.0.contains(&other_entity) {
            // If we were already allowing a collision for a particular entity,
            // and if it is penetrating us still, continue to allow it to do so.
            if any_penetrating(contacts) {
                return false;
            } else {
                // If it's no longer penetrating us, forget it.
                one_way_platform.0.remove(&other_entity);
            }
        }

        match other_colliders_query.get(other_entity) {
            // Pass-through is set to never, so accept the collision.
            Ok(Some(PassThroughOneWayPlatform::Never)) => true,
            // Pass-through is set to always, so always ignore this collision
            // and register it as an entity that's currently penetrating.
            Ok(Some(PassThroughOneWayPlatform::Always)) => {
                one_way_platform.0.insert(other_entity);
                false
            }
            // Default behaviour is "by normal".
            Err(_) | Ok(None) | Ok(Some(PassThroughOneWayPlatform::ByNormal)) => {
                // If all contact normals are in line with the local up vector of this platform,
                // then this collision should occur: the entity is on top of the platform.
                if contacts.manifolds.iter().all(|manifold| {
                    let normal = match relevant_normal {
                        RelevantNormal::Normal1 => manifold.normal1,
                        RelevantNormal::Normal2 => manifold.normal2,
                    };

                    normal.length() > Scalar::EPSILON && normal.dot(Vector::Y) >= 0.5
                }) {
                    true
                } else if any_penetrating(contacts) {
                    // If it's already penetrating, ignore the collision and register
                    // the other entity as one that's currently penetrating.
                    one_way_platform.0.insert(other_entity);
                    false
                } else {
                    // In all other cases, allow this collision.
                    true
                }
            }
        }
    });
}
