use core::marker::PhantomData;

use crate::{
    collision::contact_types::ContactId,
    data_structures::pair_key::PairKey,
    dynamics::solver::{constraint_graph::ConstraintGraph, xpbd::EntityConstraint},
    prelude::{
        ContactGraph, Joint, JointCollisionDisabled, JointDisabled, PhysicsSchedule,
        PhysicsStepSet, RigidBodyColliders,
        joint_graph::{JointGraph, JointGraphEdge},
    },
};
use bevy::{
    ecs::{
        component::{ComponentId, HookContext},
        entity_disabling::Disabled,
        world::DeferredWorld,
    },
    prelude::*,
};

/// A plugin that manages the [`JointGraph`] for a specific [joint] type.
///
/// [joint]: crate::dynamics::solver::joints
pub struct JointGraphPlugin<T: Joint + EntityConstraint<2>>(PhantomData<T>);

impl<T: Joint + EntityConstraint<2>> Default for JointGraphPlugin<T> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

/// A component that holds the [`ComponentId`] of the [joint] component on this entity, if any.
///
/// [joint]: crate::dynamics::solver::joints
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
pub struct JointComponentId(Option<ComponentId>);

impl JointComponentId {
    /// Creates a new [`JointComponentId`] component with no active joint.
    pub fn new() -> Self {
        Self(None)
    }

    /// Returns the [`ComponentId`] of the active joint component, if any.
    pub fn id(&self) -> Option<ComponentId> {
        self.0
    }
}

#[derive(Resource, Default)]
struct JointGraphPluginInitialized;

impl<T: Joint + EntityConstraint<2>> Plugin for JointGraphPlugin<T> {
    fn build(&self, app: &mut App) {
        let already_initialized = app
            .world()
            .is_resource_added::<JointGraphPluginInitialized>();

        app.init_resource::<JointGraph>();
        app.init_resource::<JointGraphPluginInitialized>();

        // Automatically add the `JointComponentId` component when the joint is added.
        app.register_required_components::<T, JointComponentId>();

        // Register hooks for adding and removing joints.
        app.world_mut()
            .register_component_hooks::<T>()
            .on_add(on_add_joint::<T>)
            .on_remove(on_remove_joint);

        if !already_initialized {
            // Remove the joint from the joint graph when it is disabled.
            app.add_observer(
                |trigger: Trigger<OnAdd, (Disabled, JointDisabled)>,
                 mut joint_graph: ResMut<JointGraph>| {
                    let entity = trigger.target();
                    joint_graph.remove_joint(entity);
                },
            );

            // Remove contacts between bodies when the `JointCollisionDisabled` component is added.
            app.add_observer(on_disable_joint_collision);
        }

        // TODO: Deduplicate these observers.
        // Add the joint back to the joint graph when `Disabled` is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, Disabled>,
             query: Query<
                (&T, Has<JointCollisionDisabled>),
                (
                    With<JointComponentId>,
                    Or<(With<Disabled>, Without<Disabled>)>,
                    Without<JointDisabled>,
                ),
            >,
             mut joint_graph: ResMut<JointGraph>| {
                let entity = trigger.target();

                // If the entity has a joint component, re-add it to the joint graph.
                if let Ok((joint, collision_disabled)) = query.get(entity) {
                    let [body1, body2] = joint.entities();
                    let joint_edge = JointGraphEdge {
                        entity,
                        collision_disabled,
                    };
                    joint_graph.add_joint(body1, body2, joint_edge);
                }
            },
        );

        // Add the joint back to the joint graph when `JointDisabled` is removed.
        app.add_observer(
            |trigger: Trigger<OnRemove, JointDisabled>,
             query: Query<(&T, Has<JointCollisionDisabled>), With<JointComponentId>>,
             mut joint_graph: ResMut<JointGraph>| {
                let entity = trigger.target();

                // If the entity has a joint component, re-add it to the joint graph.
                if let Ok((joint, collision_disabled)) = query.get(entity) {
                    let [body1, body2] = joint.entities();
                    let joint_edge = JointGraphEdge {
                        entity,
                        collision_disabled,
                    };
                    joint_graph.add_joint(body1, body2, joint_edge);
                }
            },
        );

        app.add_systems(
            PhysicsSchedule,
            on_change_joint_entities::<T>
                .in_set(PhysicsStepSet::First)
                .ambiguous_with(PhysicsStepSet::First),
        );
    }
}

fn on_add_joint<T: Joint + EntityConstraint<2>>(mut world: DeferredWorld, ctx: HookContext) {
    let entity = ctx.entity;
    let component_id = ctx.component_id;

    let mut joint = world.get_mut::<JointComponentId>(entity).unwrap();
    let old_joint = joint.0;

    // Update the joint component with the new component ID.
    joint.0 = Some(component_id);

    if let Some(old_joint) = old_joint {
        // Joint already exists, remove the old one.
        world.commands().entity(entity).remove_by_id(old_joint);

        #[cfg(debug_assertions)]
        {
            use disqualified::ShortName;

            // Log a warning about the joint replacement in case it was not intentional.
            let components = world.components();
            let old_joint_name = ShortName(components.get_info(old_joint).unwrap().name());
            let new_joint_name = ShortName(components.get_info(component_id).unwrap().name());

            warn!(
                "{old_joint_name} was replaced with {new_joint_name} on entity {entity}. An entity can only hold one joint type at a time."
            );
        }
    }

    // Add the joint to the joint graph.
    let entity_ref = world.entity(entity);
    let contacts_enabled = entity_ref.contains::<JointCollisionDisabled>();
    let joint = entity_ref.get::<T>().unwrap();
    let [body1, body2] = joint.entities();
    let joint_edge = JointGraphEdge {
        entity,
        collision_disabled: contacts_enabled,
    };
    let mut joint_graph = world.resource_mut::<JointGraph>();
    joint_graph.add_joint(body1, body2, joint_edge);
}

fn on_remove_joint(mut world: DeferredWorld, ctx: HookContext) {
    let entity = ctx.entity;
    let component_id = ctx.component_id;

    // Remove the `JointComponentId` from the entity unless the component ID
    // was changed, implying that the joint is being replaced by another one.
    if let Some(mut joint) = world.get_mut::<JointComponentId>(entity) {
        if joint.0 == Some(component_id) {
            joint.0 = None;
            // Remove the joint component.
            world
                .commands()
                .entity(entity)
                .try_remove::<JointComponentId>();

            // Remove the joint from the joint graph.
            let mut joint_graph = world.resource_mut::<JointGraph>();
            joint_graph.remove_joint(entity);
        }
    }
}

fn on_disable_joint_collision(
    trigger: Trigger<OnAdd, JointCollisionDisabled>,
    query: Query<&RigidBodyColliders>,
    joint_graph: Res<JointGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    mut constraint_graph: ResMut<ConstraintGraph>,
) {
    let entity = trigger.target();

    // Iterate through each collider of the body with fewer colliders,
    // find contacts with the other body, and remove them.
    let Some([body1, body2]) = joint_graph.bodies_of(entity) else {
        return;
    };
    let Ok([colliders1, colliders2]) = query.get_many([body1, body2]) else {
        return;
    };

    let (colliders, other_body) = if colliders1.len() < colliders2.len() {
        (colliders1, body2)
    } else {
        (colliders2, body1)
    };

    let contacts_to_remove: Vec<(ContactId, usize)> = colliders
        .iter()
        .flat_map(|collider| {
            contact_graph
                .contact_edges_with(collider)
                .filter_map(|edge| {
                    if edge.body1 == Some(other_body) || edge.body2 == Some(other_body) {
                        Some((edge.id, edge.constraint_handles.len()))
                    } else {
                        None
                    }
                })
        })
        .collect();

    for (contact_id, num_constraints) in contacts_to_remove {
        // Remove the contact from the constraint graph.
        for _ in 0..num_constraints {
            constraint_graph.pop_manifold(&mut contact_graph.edges, contact_id, body1, body2);
        }

        // Remove the contact from the contact graph.
        let pair_key = PairKey::new(body1.index(), body2.index());
        contact_graph.remove_edge_by_id(&pair_key, contact_id);
    }
}

/// Update the joint graph when the entities of a joint change.
fn on_change_joint_entities<T: Joint + EntityConstraint<2>>(
    query: Query<(Entity, &T), Changed<T>>,
    mut joint_graph: ResMut<JointGraph>,
) {
    for (entity, joint) in &query {
        let [body1, body2] = joint.entities();
        let Some([old_body1, old_body2]) = joint_graph.bodies_of(entity) else {
            continue;
        };

        if body1 != old_body1 || body2 != old_body2 {
            // Remove the old joint edge.
            if let Some(edge) = joint_graph.remove_joint(entity) {
                // Add the joint edge with the new bodies.
                joint_graph.add_joint(body1, body2, edge);
            }
        }
    }
}

// TODO: Tests
