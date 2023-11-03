use crate::prelude::*;
use bevy::prelude::*;

/// Controls the global physics debug configuration. See [`PhysicsDebugPlugin`]
///
/// To configure the debug rendering of specific entities, use the [`DebugRender`] component.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]

/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin::default(),
///         ))
///         // Overwrite default debug configuration (optional)
///         .insert_resource(PhysicsDebugConfig {
///             aabb_color: Some(Color::WHITE),
///             ..default()
///         })
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
///     commands.spawn((RigidBody::Dynamic, Collider::ball(0.5)));
/// }
/// ```
#[derive(Reflect, Resource)]
#[reflect(Resource)]
pub struct PhysicsDebugConfig {
    /// Determines if debug rendering is enabled.
    pub enabled: bool,
    /// The lengths of the axes drawn for an entity at the center of mass.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABBs](ColliderAabb). If `None`, the AABBs will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframes. If `None`, the colliders will not be rendered.
    pub collider_color: Option<Color>,
    /// The colors (in HSLA) for [sleeping](Sleeping) bodies will be multiplied by this array.
    /// If `None`, sleeping will have no effect on the colors.
    pub sleeping_color_multiplier: Option<[f32; 4]>,
    /// The color of the contact points. If `None`, the contact points will not be rendered.
    pub contact_color: Option<Color>,
    /// The color of the lines drawn from the centers of bodies to their joint anchors.
    pub joint_anchor_color: Option<Color>,
    /// The color of the lines drawn between joint anchors, indicating the separation.
    pub joint_separation_color: Option<Color>,
    /// The color used for the rays in [raycasts](spatial_query#raycasting).
    pub raycast_color: Option<Color>,
    /// The color used for the hit points in [raycasts](spatial_query#raycasting).
    pub raycast_point_color: Option<Color>,
    /// The color used for the hit normals in [raycasts](spatial_query#raycasting).
    pub raycast_normal_color: Option<Color>,
    /// The color used for the ray in [shapecasts](spatial_query#shapecasting).
    pub shapecast_color: Option<Color>,
    /// The color used for the shape in [shapecasts](spatial_query#shapecasting).
    pub shapecast_shape_color: Option<Color>,
    /// The color used for the hit points in [shapecasts](spatial_query#shapecasting).
    pub shapecast_point_color: Option<Color>,
    /// The color used for the hit normals in [shapecasts](spatial_query#shapecasting).
    pub shapecast_normal_color: Option<Color>,
    /// Determines if the visibility of entities with [colliders](Collider) should be set to `Visibility::Hidden`,
    /// which will only show the debug renders.
    pub hide_meshes: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(Color::ORANGE),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            contact_color: None,
            joint_anchor_color: Some(Color::PINK),
            joint_separation_color: Some(Color::RED),
            raycast_color: Some(Color::RED),
            raycast_point_color: Some(Color::YELLOW),
            raycast_normal_color: Some(Color::PINK),
            shapecast_color: Some(Color::RED),
            shapecast_shape_color: Some(Color::rgb(0.4, 0.6, 1.0)),
            shapecast_point_color: Some(Color::YELLOW),
            shapecast_normal_color: Some(Color::PINK),
            hide_meshes: false,
        }
    }
}

impl PhysicsDebugConfig {
    /// Creates a [`PhysicsDebugConfig`] configuration with all rendering options enabled.
    pub fn all() -> Self {
        Self {
            enabled: true,
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: Some(Color::rgb(0.8, 0.8, 0.8)),
            collider_color: Some(Color::ORANGE),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            contact_color: Some(Color::CYAN),
            joint_anchor_color: Some(Color::PINK),
            joint_separation_color: Some(Color::RED),
            raycast_color: Some(Color::RED),
            raycast_point_color: Some(Color::YELLOW),
            raycast_normal_color: Some(Color::PINK),
            shapecast_color: Some(Color::RED),
            shapecast_shape_color: Some(Color::rgb(0.4, 0.6, 1.0)),
            shapecast_point_color: Some(Color::YELLOW),
            shapecast_normal_color: Some(Color::PINK),
            hide_meshes: true,
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with debug rendering enabled but all options turned off.
    ///
    /// Note: this doesn't affect entities with [`DebugRender`] component; their debug gizmos will be visible.
    pub fn none() -> Self {
        Self {
            enabled: true,
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            sleeping_color_multiplier: None,
            contact_color: None,
            joint_anchor_color: None,
            joint_separation_color: None,
            raycast_color: None,
            raycast_point_color: None,
            raycast_normal_color: None,
            shapecast_color: None,
            shapecast_shape_color: None,
            shapecast_point_color: None,
            shapecast_normal_color: None,
            hide_meshes: false,
        }
    }

    /// Creates a [`PhysicsDebugConfig`] configuration with the given lengths for the axes
    /// that are drawn for the entity at the center of mass. Other debug rendering options will be disabled.
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
    pub fn joints(anchor_color: Option<Color>, separation_color: Option<Color>) -> Self {
        Self {
            joint_anchor_color: anchor_color,
            joint_separation_color: separation_color,
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

    /// Sets the multiplier used for the colors (in HSLA) of [sleeping](Sleeping) bodies.
    pub fn with_sleeping_color_multiplier(mut self, color_multiplier: [f32; 4]) -> Self {
        self.sleeping_color_multiplier = Some(color_multiplier);
        self
    }

    /// Sets the contact color.
    pub fn with_contact_color(mut self, color: Color) -> Self {
        self.contact_color = Some(color);
        self
    }

    /// Sets the colors used for debug rendering joints.
    pub fn with_joint_colors(anchor_color: Option<Color>, separation_color: Option<Color>) -> Self {
        Self {
            joint_anchor_color: anchor_color,
            joint_separation_color: separation_color,
            ..Self::none()
        }
    }

    /// Sets the colors used for debug rendering raycasts.
    pub fn with_raycast_colors(
        mut self,
        ray: Option<Color>,
        hit_point: Option<Color>,
        hit_normal: Option<Color>,
    ) -> Self {
        self.raycast_color = ray;
        self.raycast_point_color = hit_point;
        self.raycast_normal_color = hit_normal;
        self
    }

    /// Sets the colors used for debug rendering shapecasts.
    pub fn with_shapecast_colors(
        mut self,
        ray: Option<Color>,
        shape: Option<Color>,
        hit_point: Option<Color>,
        hit_normal: Option<Color>,
    ) -> Self {
        self.shapecast_color = ray;
        self.shapecast_shape_color = shape;
        self.shapecast_point_color = hit_point;
        self.shapecast_normal_color = hit_normal;
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

    /// Disables raycast debug rendering.
    pub fn without_raycasts(mut self) -> Self {
        self.raycast_color = None;
        self.raycast_point_color = None;
        self.raycast_normal_color = None;
        self
    }

    /// Disables shapecast debug rendering.
    pub fn without_shapecasts(mut self) -> Self {
        self.shapecast_color = None;
        self.shapecast_shape_color = None;
        self.shapecast_point_color = None;
        self.shapecast_normal_color = None;
        self
    }
}

/// A component for the debug render configuration of an entity. See [`PhysicsDebugPlugin`].
///
/// This overwrites the global [`PhysicsDebugConfig`] for this specific entity.
///
/// ## Example
///
/// ```no_run
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin::default(),
///         ))
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::ball(0.5),
///         // Overwrite default collider color (optional)
///         DebugRender::default().with_collider_color(Color::RED),
///     ));
/// }
/// ```
#[derive(Component, Reflect, Clone, Copy, PartialEq)]
#[reflect(Component)]
pub struct DebugRender {
    /// The lengths of the axes drawn for the entity at the center of mass.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABB](ColliderAabb). If `None`, the AABB will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframe. If `None`, the collider will not be rendered.
    pub collider_color: Option<Color>,
    /// If the entity is [sleeping](Sleeping), its colors (in HSLA) will be multiplied by this array.
    /// If `None`, sleeping will have no effect on the colors.
    pub sleeping_color_multiplier: Option<[f32; 4]>,
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
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
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
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            hide_mesh: true,
        }
    }

    /// Disables all debug rendering for this entity.
    pub fn none() -> Self {
        Self {
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            sleeping_color_multiplier: None,
            hide_mesh: false,
        }
    }

    /// Creates a [`DebugRender`] configuration with the given lengths for the axes
    /// that are drawn for the entity at the center of mass. Other debug rendering options will be disabled.
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

    /// Sets the lengths of the axes drawn for the entity at the center of mass.
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

    /// Sets the multiplier used for the colors (in HSLA) of [sleeping](Sleeping) bodies.
    pub fn with_sleeping_color_multiplier(mut self, color_multiplier: [f32; 4]) -> Self {
        self.sleeping_color_multiplier = Some(color_multiplier);
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
