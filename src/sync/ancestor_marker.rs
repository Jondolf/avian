//! Functionality for marking ancestors of entities with marker components.

use std::marker::PhantomData;

use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    hierarchy::HierarchyEvent,
    prelude::*,
};

/// A plugin that marks the ancestors of entities that have the given component `C`
/// with the [`AncestorMarker`] component.
///
/// One use case is speeding up transform propagation: we only need to propagate
/// down trees that have a certain type of entity, like a collider or a rigid body.
pub struct AncestorMarkerPlugin<C: Component> {
    schedule: Interned<dyn ScheduleLabel>,
    system_set: Option<Interned<dyn SystemSet>>,
    _phantom: PhantomData<C>,
}

impl<C: Component> AncestorMarkerPlugin<C> {
    /// Creates a new [`AncestorMarkerPlugin`] with the schedule that the system
    /// adding markers should run in.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            system_set: None,
            _phantom: PhantomData,
        }
    }

    /// Configures which system set the system adding the [`AncestorMarker`]s should run in.
    ///
    /// Note: Unlike the normal `in_set` for system configurations, this *overwrites* the set,
    /// so the system can only be added to a single system set at a time.
    pub fn add_markers_in_set(mut self, set: impl SystemSet) -> Self {
        self.system_set = Some(set.intern());
        self
    }
}

impl<C: Component> Plugin for AncestorMarkerPlugin<C> {
    fn build(&self, app: &mut App) {
        // Add `AncestorMarker<C>` for the ancestors of added colliders,
        // until an ancestor that has other `AncestorMarker<C>` entities as children is encountered.
        app.observe(
            |trigger: Trigger<OnAdd, C>,
             mut commands: Commands,
             parent_query: Query<&Parent>,
             ancestor_query: Query<(), With<AncestorMarker<C>>>| {
                let entity = trigger.entity();
                if parent_query.contains(entity) {
                    add_ancestor_markers(
                        entity,
                        &mut commands,
                        &parent_query,
                        &ancestor_query,
                        false,
                    );
                }
            },
        );

        // Remove `AncestorMarker<C>` from removed colliders and their ancestors,
        // until an ancestor that has other `AncestorMarker<C>` entities as children is encountered.
        #[allow(clippy::type_complexity)]
        app.observe(
            |trigger: Trigger<OnRemove, C>,
            mut commands: Commands,
            child_query: Query<&Children>,
            parent_query: Query<&Parent>,
            ancestor_query: Query<
                (Entity, Has<C>),
                Or<(With<AncestorMarker<C>>, With<C>)>
            >| {
                remove_ancestor_markers(trigger.entity(), &mut commands, &parent_query, &child_query, &ancestor_query, false);
            },
        );

        // Initialize `HierarchyEvent` in case `HierarchyPlugin` is not added.
        app.add_event::<HierarchyEvent>();

        // Update markers when changes are made to the hierarchy.
        // TODO: This should be an observer. It'd remove the need for this scheduling nonsense
        //       and make the implementation more robust.
        if let Some(set) = self.system_set {
            app.add_systems(
                self.schedule,
                update_markers_on_hierarchy_changes::<C>.in_set(set),
            );
        } else {
            app.add_systems(self.schedule, update_markers_on_hierarchy_changes::<C>);
        }
    }
}

/// A marker component that marks an entity as an ancestor of an entity with the given component `C`.
///
/// This is added and removed automatically by the [`AncestorMarkerPlugin`] if it is enabled.
#[derive(Component, Reflect)]
pub struct AncestorMarker<C: Component> {
    _phantom: PhantomData<C>,
}

impl<C: Component> Default for AncestorMarker<C> {
    fn default() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

// TODO: This should be an observer once there is a trigger for hierarchy changes.
//       See https://github.com/bevyengine/bevy/pull/13925 for an implementation of that trigger.
/// Marks ancestors of entities that have the given component `C` with the `AncestorMarker<C>` component.
#[allow(clippy::type_complexity)]
fn update_markers_on_hierarchy_changes<C: Component>(
    mut commands: Commands,
    entity_query: Query<(), With<C>>,
    parent_query: Query<&Parent>,
    child_query: Query<&Children>,
    ancestor_query_1: Query<(), With<AncestorMarker<C>>>,
    ancestor_query_2: Query<(Entity, Has<C>), Or<(With<AncestorMarker<C>>, With<C>)>>,
    mut hierarchy_event: EventReader<HierarchyEvent>,
) {
    for event in hierarchy_event.read().cloned() {
        match event {
            HierarchyEvent::ChildAdded { child, parent } => {
                if entity_query.contains(child) {
                    // Mark the child's ancestors.
                    add_ancestor_markers(
                        parent,
                        &mut commands,
                        &parent_query,
                        &ancestor_query_1,
                        true,
                    );
                }
            }

            HierarchyEvent::ChildRemoved { child, parent } => {
                if entity_query.contains(child) {
                    // Remove markers from the parent and its ancestors.
                    remove_ancestor_markers(
                        parent,
                        &mut commands,
                        &parent_query,
                        &child_query,
                        &ancestor_query_2,
                        true,
                    );
                }
            }

            HierarchyEvent::ChildMoved {
                child,
                previous_parent,
                new_parent,
            } => {
                if entity_query.contains(child) {
                    // Remove markers from the previous parent and its ancestors.
                    remove_ancestor_markers(
                        previous_parent,
                        &mut commands,
                        &parent_query,
                        &child_query,
                        &ancestor_query_2,
                        true,
                    );

                    // Mark the new parent and its ancestors.
                    add_ancestor_markers(
                        new_parent,
                        &mut commands,
                        &parent_query,
                        &ancestor_query_1,
                        true,
                    );
                }
            }
        }
    }
}

fn add_ancestor_markers<C: Component>(
    entity: Entity,
    commands: &mut Commands,
    parent_query: &Query<&Parent>,
    ancestor_query: &Query<(), With<AncestorMarker<C>>>,
    include_self: bool,
) {
    if include_self {
        commands
            .entity(entity)
            .insert(AncestorMarker::<C>::default());
    }

    // Traverse up the tree, marking entities with `AncestorMarker<C>`
    // until an entity that already has it is encountered.
    for parent_entity in parent_query.iter_ancestors(entity) {
        if ancestor_query.contains(parent_entity) {
            break;
        } else {
            commands
                .entity(parent_entity)
                .insert(AncestorMarker::<C>::default());
        }
    }
}

/// Remove the component from entity, unless it is already despawned.
fn remove_component<T: Bundle>(commands: &mut Commands, entity: Entity) {
    if let Some(mut entity_commands) = commands.get_entity(entity) {
        entity_commands.remove::<T>();
    }
}

#[allow(clippy::type_complexity)]
fn remove_ancestor_markers<C: Component>(
    entity: Entity,
    commands: &mut Commands,
    parent_query: &Query<&Parent>,
    child_query: &Query<&Children>,
    ancestor_query: &Query<(Entity, Has<C>), Or<(With<AncestorMarker<C>>, With<C>)>>,
    include_self: bool,
) {
    if include_self {
        // Remove the marker from the `parent` unless a sibling of the `child`
        // is also a marked ancestor or has `C`.
        if let Ok(children) = child_query.get(entity) {
            let keep_marker = ancestor_query
                .iter_many(children)
                .any(|(parent_child, _has_c)| parent_child != entity);
            if keep_marker {
                return;
            } else {
                remove_component::<AncestorMarker<C>>(commands, entity);
            }
        } else {
            // The parent has no children, so it cannot be an ancestor.
            remove_component::<AncestorMarker<C>>(commands, entity);
        }
    }

    // Iterate over ancestors, removing `AncestorMarker<C>` markers until
    // an entity that has other `AncestorMarker<C>` children is encountered.
    let mut previous_parent = entity;
    for parent_entity in parent_query.iter_ancestors(entity) {
        if let Ok(children) = child_query.get(parent_entity) {
            // Keep the marker if `parent_entity` has a child that is a marked ancestor
            // or an entity that has `C`, but not the one that was removed.
            let keep_marker = ancestor_query
                .iter_many(children)
                .any(|(child, has_c)| child != previous_parent || (has_c && child != entity));

            if keep_marker {
                return;
            } else {
                remove_component::<AncestorMarker<C>>(commands, parent_entity);
            }
        } else {
            // The parent has no children, so it cannot be an ancestor.
            remove_component::<AncestorMarker<C>>(commands, entity);
        }

        previous_parent = parent_entity;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Component)]
    struct C;

    #[test]
    fn add_and_remove_component() {
        let mut app = App::new();

        app.add_plugins((AncestorMarkerPlugin::<C>::new(PostUpdate), HierarchyPlugin));

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn_empty().set_parent(an).id();
        let cy = app.world_mut().spawn(C).set_parent(an).id();

        let dn = app.world_mut().spawn_empty().set_parent(cy).id();
        let en = app.world_mut().spawn_empty().set_parent(cy).id();

        let fy = app.world_mut().spawn(C).set_parent(dn).id();
        let gy = app.world_mut().spawn(C).set_parent(dn).id();

        app.world_mut().run_schedule(PostUpdate);

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Remove `C` from FY. DN, CY, and AN should all keep the `AncestorMarker<C>` marker.
        let mut entity_mut = app.world_mut().entity_mut(fy);
        entity_mut.remove::<C>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Remove `C` from GY. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY, but it should remain on AN.
        let mut entity_mut = app.world_mut().entity_mut(gy);
        entity_mut.remove::<C>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Remove `C` from CY. The `AncestorMarker<C>` marker should
        // now be removed from AN.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.remove::<C>();

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(an).contains::<AncestorMarker<C>>());
    }

    #[test]
    fn remove_children() {
        let mut app = App::new();

        app.add_plugins((AncestorMarkerPlugin::<C>::new(PostUpdate), HierarchyPlugin));

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn_empty().set_parent(an).id();
        let cy = app.world_mut().spawn(C).set_parent(an).id();

        let dn = app.world_mut().spawn_empty().set_parent(cy).id();
        let en = app.world_mut().spawn_empty().set_parent(cy).id();

        let fy = app.world_mut().spawn(C).set_parent(dn).id();
        let gy = app.world_mut().spawn(C).set_parent(dn).id();

        app.world_mut().run_schedule(PostUpdate);

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Make FY an orphan.
        let mut entity_mut = app.world_mut().entity_mut(dn);
        entity_mut.remove_children(&[fy]);

        app.world_mut().run_schedule(PostUpdate);

        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make GY an orphan. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY, but it should remain on AN.
        let mut entity_mut = app.world_mut().entity_mut(dn);
        entity_mut.remove_children(&[gy]);

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY an orphan. The `AncestorMarker<C>` marker should
        // now be removed from AN.
        let mut entity_mut = app.world_mut().entity_mut(an);
        entity_mut.remove_children(&[cy]);

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY a child of AN again. The `AncestorMarker<C>` marker should
        // now be added to AN.
        let mut entity_mut = app.world_mut().entity_mut(an);
        entity_mut.add_child(cy);

        app.world_mut().run_schedule(PostUpdate);
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY an orphan and delete AN. This must not crash.
        let mut entity_mut = app.world_mut().entity_mut(an);
        entity_mut.remove_children(&[cy]);
        entity_mut.despawn();

        app.world_mut().run_schedule(PostUpdate);
    }

    #[test]
    fn move_children() {
        let mut app = App::new();

        app.add_plugins((AncestorMarkerPlugin::<C>::new(PostUpdate), HierarchyPlugin));

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn_empty().set_parent(an).id();
        let cy = app.world_mut().spawn(C).set_parent(an).id();

        let dn = app.world_mut().spawn_empty().set_parent(cy).id();
        let en = app.world_mut().spawn_empty().set_parent(cy).id();

        let fy = app.world_mut().spawn(C).set_parent(dn).id();
        let gy = app.world_mut().spawn(C).set_parent(dn).id();

        app.world_mut().run_schedule(PostUpdate);

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Move FY to be a child of BN. BN should get the `AncestorMarker<C>` component.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.add_child(fy);

        app.world_mut().run_schedule(PostUpdate);

        assert!(app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move GY to be a child of BN. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.add_child(gy);

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move all children from BN to CY. The `AncestorMarker<C>` marker should
        // now be removed from BN, and CY should get it back.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.push_children(&[fy, gy]);

        app.world_mut().run_schedule(PostUpdate);

        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move all children from CY to BN and remove CY. This must not crash.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.push_children(&[dn, en, fy, gy]);

        app.world_mut().run_schedule(PostUpdate);
    }
}
