use crate::prelude::*;
use bevy::prelude::*;

/// Controls the global physics debug configuration.
///
/// To configure the debug rendering of specific entities, use the [`DebugRender`] component.
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct PhysicsDebugConfig {
    /// The lengths of the axes drawn for an entity.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABBs](ColliderAabb). If `None`, the AABBs will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframes. If `None`, the colliders will not be rendered.
    pub collider_color: Option<Color>,
    /// The color of the contact points. If `None`, the contact points will not be rendered.
    pub contact_color: Option<Color>,
    /// The color of the lines drawn from the centers of bodies to their joint anchors.
    pub joint_anchor_color: Option<Color>,
    /// The color of the lines drawn between joint anchors, indicating the separation.
    pub joint_separation_color: Option<Color>,
    /// Determines if the visibility of entities with [colliders](Collider) should be set to `Visibility::Hidden`,
    /// which will only show the debug renders.
    pub hide_meshes: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(Color::ORANGE),
            contact_color: None,
            joint_anchor_color: Some(Color::PINK),
            joint_separation_color: Some(Color::RED),
            hide_meshes: false,
        }
    }
}

impl PhysicsDebugConfig {
    /// Creates a [`PhysicsDebugConfig`] configuration with all rendering options enabled.
    pub fn all() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: Some(Color::rgb(0.8, 0.8, 0.8)),
            collider_color: Some(Color::ORANGE),
            contact_color: Some(Color::CYAN),
            joint_anchor_color: Some(Color::PINK),
            joint_separation_color: Some(Color::RED),
            hide_meshes: true,
        }
    }

    /// Disables all debug rendering for this entity.
    pub fn none() -> Self {
        Self {
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            contact_color: None,
            joint_anchor_color: None,
            joint_separation_color: None,
            hide_meshes: false,
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with the given lengths for the axes
    /// that are drawn for the entity. Other debug rendering options will be disabled.
    pub fn axes(axis_lengths: Vector) -> Self {
        Self {
            axis_lengths: Some(axis_lengths),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with a given AABB color.
    /// Other debug rendering options will be disabled.
    pub fn aabbs(color: Color) -> Self {
        Self {
            aabb_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with a given collider color.
    /// Other debug rendering options will be disabled.
    pub fn colliders(color: Color) -> Self {
        Self {
            collider_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with a given contact color.
    /// Other debug rendering options will be disabled.
    pub fn contacts(color: Color) -> Self {
        Self {
            contact_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with given colors for
    /// joint anchors and separation distances. Other debug rendering options will be disabled.
    pub fn joints(anchor_color: Color, separation_color: Color) -> Self {
        Self {
            joint_anchor_color: Some(anchor_color),
            joint_separation_color: Some(separation_color),
            ..Self::none()
        }
    }

    /// Sets the lengths of the axes drawn for the entity.
    pub fn with_axes(mut self, axis_lengths: Vector) -> Self {
        self.axis_lengths = Some(axis_lengths);
        self
    }

    /// Sets the AABB color.
    pub fn with_aabb_color(mut self, color: Color) -> Self {
        self.aabb_color = Some(color);
        self
    }

    /// Sets the collider color.
    pub fn with_collider_color(mut self, color: Color) -> Self {
        self.collider_color = Some(color);
        self
    }

    /// Sets the contact color.
    pub fn with_contact_color(mut self, color: Color) -> Self {
        self.contact_color = Some(color);
        self
    }

    /// Sets the visibility of the entity's visual mesh.
    pub fn with_mesh_visibility(mut self, is_visible: bool) -> Self {
        self.hide_meshes = !is_visible;
        self
    }

    /// Disables axis debug rendering.
    pub fn without_axes(mut self) -> Self {
        self.axis_lengths = None;
        self
    }

    /// Disables AABB debug rendering.
    pub fn without_aabbs(mut self) -> Self {
        self.aabb_color = None;
        self
    }

    /// Disables collider debug rendering.
    pub fn without_colliders(mut self) -> Self {
        self.collider_color = None;
        self
    }

    /// Disables contact debug rendering.
    pub fn without_contacts(mut self) -> Self {
        self.contact_color = None;
        self
    }

    /// Disables joint debug rendering.
    pub fn without_joints(mut self) -> Self {
        self.joint_anchor_color = None;
        self.joint_separation_color = None;
        self
    }
}

/// A component for the debug render configuration of an entity.
///
/// This overwrites the global [`PhysicsDebugConfig`] for this specific entity.
#[derive(Component, Reflect, Clone, Copy, PartialEq)]
#[reflect(Component)]
pub struct DebugRender {
    /// The lengths of the axes drawn for the entity.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABB](ColliderAabb). If `None`, the AABB will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframe. If `None`, the collider will not be rendered.
    pub collider_color: Option<Color>,
    /// Determines if the entity's visibility should be set to `Visibility::Hidden`, which will only show the debug render.
    pub hide_mesh: bool,
}

impl Default for DebugRender {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(Color::ORANGE),
            hide_mesh: false,
        }
    }
}

impl DebugRender {
    /// Creates a [`DebugRender`] configuration with all rendering options enabled.
    pub fn all() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: Some(Color::rgb(0.8, 0.8, 0.8)),
            collider_color: Some(Color::ORANGE),
            hide_mesh: true,
        }
    }

    /// Disables all debug rendering for this entity.
    pub fn none() -> Self {
        Self {
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            hide_mesh: false,
        }
    }

    /// Creates a [`DebugRender`] configuration with the given lengths for the axes
    /// that are drawn for the entity. Other debug rendering options will be disabled.
    pub fn axes(axis_lengths: Vector) -> Self {
        Self {
            axis_lengths: Some(axis_lengths),
            ..Self::none()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given AABB color.
    /// Other debug rendering options will be disabled.
    pub fn aabb(color: Color) -> Self {
        Self {
            aabb_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given collider color.
    /// Other debug rendering options will be disabled.
    pub fn collider(color: Color) -> Self {
        Self {
            collider_color: Some(color),
            ..Self::none()
        }
    }

    /// Sets the lengths of the axes drawn for the entity.
    pub fn with_axes(mut self, axis_lengths: Vector) -> Self {
        self.axis_lengths = Some(axis_lengths);
        self
    }

    /// Sets the AABB color.
    pub fn with_aabb_color(mut self, color: Color) -> Self {
        self.aabb_color = Some(color);
        self
    }

    /// Sets the collider color.
    pub fn with_collider_color(mut self, color: Color) -> Self {
        self.collider_color = Some(color);
        self
    }

    /// Sets the visibility of the entity's visual mesh.
    pub fn with_mesh_visibility(mut self, is_visible: bool) -> Self {
        self.hide_mesh = !is_visible;
        self
    }

    /// Disables axis debug rendering.
    pub fn without_axes(mut self) -> Self {
        self.axis_lengths = None;
        self
    }

    /// Disables AABB debug rendering.
    pub fn without_aabb(mut self) -> Self {
        self.aabb_color = None;
        self
    }

    /// Disables collider debug rendering.
    pub fn without_collider(mut self) -> Self {
        self.collider_color = None;
        self
    }
}
