use crate::prelude::*;
use bevy::prelude::*;
use bevy::utils::HashMap;

/// A component that will automatically generate [`Collider`]s on its descendants at runtime.
/// The type of the generated collider can be specified using [`ColliderConstructor`].
/// This supports computing the shape dynamically from the mesh, in which case only the descendants
/// with a [`Mesh`] will have colliders generated.
///
/// In contrast to [`ColliderConstructor`], this component will *not* generate a collider on its own entity.
///
/// If this component is used on a scene, such as one spawned by a [`SceneBundle`], it will
/// wait until the scene is loaded before generating colliders.
///
/// The exact configuration for each descendant can be specified using the helper methods
/// such as [`with_constructor_for_name`](Self::with_constructor_for_name).
///
/// This component will only override a pre-existing [`Collider`] component on a descendant entity
/// when it has been explicitly mentioned in the `config`.
///
/// ## See also
///
/// For inserting colliders on the same entity, use [`ColliderConstructor`].
///
/// ## Caveats
///
/// When a component has multiple ancestors with [`ColliderConstructorHierarchy`], the insertion order is undefined.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>) {
///     let scene = assets.load("my_model.gltf#Scene0");
///
#[cfg_attr(
    feature = "2d",
    doc = "     // Spawn the scene and automatically generate circle colliders"
)]
#[cfg_attr(
    feature = "3d",
    doc = "     // Spawn the scene and automatically generate triangle mesh colliders"
)]
///
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
#[cfg_attr(
    feature = "2d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 }),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),"
)]
///     ));
///
///     // Specify configuration for specific meshes by name
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
#[cfg_attr(
    feature = "2d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 })
             .with_constructor_for_name(\"Tree\", ColliderConstructor::Rectangle { x_length: 1.0, y_length: 2.0 })"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh)
             .with_constructor_for_name(\"Tree\", ColliderConstructor::ConvexHullFromMesh)"
)]
///             .with_layers_for_name("Tree", CollisionLayers::from_bits(0b0010, 0b1111))
///             .with_density_for_name("Tree", 2.5),
///     ));
///
///     // Only generate colliders for specific meshes by name
///     commands.spawn((
///         SceneBundle { scene: scene.clone(), ..default() },
///         ColliderConstructorHierarchy::new(None)
#[cfg_attr(
    feature = "2d",
    doc = "             .with_constructor_for_name(\"Tree\", ColliderConstructor::Circle { radius: 2.0 }),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "             .with_constructor_for_name(\"Tree\", ColliderConstructor::ConvexHullFromMesh),"
)]
///     ));
///
///     // Generate colliders for everything except specific meshes by name
///     commands.spawn((
///         SceneBundle { scene, ..default() },
#[cfg_attr(
    feature = "2d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 })"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
             TrimeshFlags::MERGE_DUPLICATE_VERTICES
         ))"
)]
///         .without_constructor_for_name("Tree"),
///     ));
/// }
/// ```
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Debug, Component, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
pub struct ColliderConstructorHierarchy {
    /// The default collider type used for each entity that isn't included in [`config`](Self::config).
    /// If `None`, all entities except the ones in [`config`](Self::config) will be skipped.
    pub default_constructor: Option<ColliderConstructor>,
    /// The default [`CollisionLayers`] used for colliders in the hierarchy.
    ///
    /// [`CollisionLayers::ALL`] by default.
    pub default_layers: CollisionLayers,
    /// The default [`ColliderDensity`] used for colliders in the hierarchy.
    ///
    /// `1.0` by default.
    pub default_density: ColliderDensity,
    /// Specifies data like the [`ColliderConstructor`] and [`CollisionLayers`] for entities
    /// in the hierarchy by `Name`. Entries with a `None` value will be skipped.
    ///
    /// For the entities not found in this `HashMap`, [`default_constructor`](Self::default_constructor),
    /// [`default_layers`](Self::default_layers), and [`default_density`](Self::default_density) will be used instead.
    pub config: HashMap<String, Option<ColliderConstructorHierarchyConfig>>,
}

impl ColliderConstructorHierarchy {
    /// Creates a new [`ColliderConstructorHierarchy`] with the default [`ColliderConstructor`] used for
    /// generating colliders set to the given `default_constructor`.
    ///
    /// If the given constructor type is `None`, collider generation is skipped
    /// for all entities in the hierarchy except the ones in [`config`](Self::config).
    ///
    /// Collider constructors can be specified for individual entities using
    /// [`with_constructor_for_name`](Self::with_constructor_for_name).
    pub fn new(default_constructor: impl Into<Option<ColliderConstructor>>) -> Self {
        Self {
            default_constructor: default_constructor.into(),
            default_layers: CollisionLayers::ALL,
            default_density: ColliderDensity(1.0),
            config: default(),
        }
    }

    /// Specifies the default [`CollisionLayers`] used for colliders not included in [`ColliderConstructorHierarchy::config`].
    pub fn with_default_layers(mut self, layers: CollisionLayers) -> Self {
        self.default_layers = layers;
        self
    }

    /// Specifies the default [`ColliderDensity`] used for colliders not included in [`ColliderConstructorHierarchy::config`].
    pub fn with_default_density(mut self, density: impl Into<ColliderDensity>) -> Self {
        self.default_density = density.into();
        self
    }

    /// Specifies the [`ColliderConstructor`] used for an entity with the given `name`.
    pub fn with_constructor_for_name(
        mut self,
        name: &str,
        constructor: ColliderConstructor,
    ) -> Self {
        if let Some(Some(data)) = self.config.get_mut(name) {
            data.constructor = Some(constructor);
        } else {
            self.config.insert(
                name.to_string(),
                Some(ColliderConstructorHierarchyConfig {
                    constructor: Some(constructor),
                    ..default()
                }),
            );
        }
        self
    }

    /// Specifies the [`CollisionLayers`] used for an entity with the given `name`.
    pub fn with_layers_for_name(self, name: &str, layers: CollisionLayers) -> Self {
        self.with_config_for_name(name, |config| config.layers = Some(layers))
    }

    /// Specifies the [`ColliderDensity`] used for an entity with the given `name`.
    pub fn with_density_for_name(self, name: &str, density: impl Into<ColliderDensity>) -> Self {
        let density = density.into();
        self.with_config_for_name(name, |config| config.density = Some(density))
    }

    /// Sets the [`ColliderConstructor`] for the entity associated with the given `name` to `None`,
    /// skipping collider generation for it.
    pub fn without_constructor_for_name(mut self, name: &str) -> Self {
        self.config.insert(name.to_string(), None);
        self
    }

    fn with_config_for_name(
        mut self,
        name: &str,
        mut mutate_config: impl FnMut(&mut ColliderConstructorHierarchyConfig),
    ) -> Self {
        if let Some(Some(config)) = self.config.get_mut(name) {
            mutate_config(config);
        } else {
            let mut config = ColliderConstructorHierarchyConfig::default();
            mutate_config(&mut config);
            self.config.insert(name.to_string(), Some(config));
        }
        self
    }
}

/// Configuration for a specific collider generated from a scene using [`ColliderConstructorHierarchy`].
#[derive(Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[cfg_attr(all(feature = "3d", feature = "collider-from-mesh"), reflect(Default))]
pub struct ColliderConstructorHierarchyConfig {
    /// The type of collider generated for the mesh.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_constructor`] is used instead.
    pub constructor: Option<ColliderConstructor>,
    /// The [`CollisionLayers`] used for this collider.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_layers`] is used instead.
    pub layers: Option<CollisionLayers>,
    /// The [`ColliderDensity`] used for this collider.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_density`] is used instead.
    pub density: Option<ColliderDensity>,
}

/// A component that will automatically generate a [`Collider`] at runtime using [`Collider::try_from_constructor`].
/// Enabling the `collider-from-mesh` feature activates support for computing the shape dynamically from the mesh attached to the same entity.
///
/// Since [`Collider`] is not [`Reflect`], you can use this type to statically statically
/// specify a collider's shape instead.
///
/// This component will never override a pre-existing [`Collider`] component on the same entity.
///
/// ## See also
///
/// For inserting colliders on an entity's descendants, use [`ColliderConstructorHierarchy`].
///
/// ## Panics
///
/// The system handling the generation of colliders will panic if the specified [`ColliderConstructor`]
/// requires a mesh, but the entity does not have a `Handle<Mesh>` component.
///
/// ## Example
///
/// ```
/// use bevy::prelude::*;
#[cfg_attr(feature = "2d", doc = "use bevy_xpbd_2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use bevy_xpbd_3d::prelude::*;")]
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>, mut meshes: Assets<Mesh>) {
#[cfg_attr(feature = "2d", doc = "      // Spawn a circle with radius 2")]
#[cfg_attr(
    feature = "3d",
    doc = "      // Spawn a cube with a convex hull collider generated from the mesh"
)]
///     commands.spawn((
#[cfg_attr(
    feature = "2d",
    doc = "         ColliderConstructor::Circle { radius: 2.0 },"
)]
#[cfg_attr(
    feature = "3d",
    doc = "         ColliderConstructor::ConvexHullFromMesh,"
)]
///         PbrBundle {
///             mesh: meshes.add(Mesh::from(Cuboid::default())),
///             ..default()
///         },
///     ));
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Reflect, Component)]
#[reflect(Debug, Component, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[cfg_attr(all(feature = "3d", feature = "collider-from-mesh"), derive(Default))]
#[cfg_attr(all(feature = "3d", feature = "collider-from-mesh"), reflect(Default))]
#[allow(missing_docs)]
pub enum ColliderConstructor {
    /// Constructs a collider with [`Collider::circle`].
    #[cfg(feature = "2d")]
    Circle { radius: Scalar },
    /// Constructs a collider with [`Collider::sphere`].
    #[cfg(feature = "3d")]
    Sphere { radius: Scalar },
    /// Constructs a collider with [`Collider::ellipse`].
    #[cfg(feature = "2d")]
    Ellipse {
        half_width: Scalar,
        half_height: Scalar,
    },
    /// Constructs a collider with [`Collider::rectangle`].
    #[cfg(feature = "2d")]
    Rectangle { x_length: Scalar, y_length: Scalar },
    /// Constructs a collider with [`Collider::cuboid`].
    #[cfg(feature = "3d")]
    Cuboid {
        x_length: Scalar,
        y_length: Scalar,
        z_length: Scalar,
    },
    /// Constructs a collider with [`Collider::round_rectangle`].
    #[cfg(feature = "2d")]
    RoundRectangle {
        x_length: Scalar,
        y_length: Scalar,
        border_radius: Scalar,
    },
    /// Constructs a collider with [`Collider::round_cuboid`].
    #[cfg(feature = "3d")]
    RoundCuboid {
        x_length: Scalar,
        y_length: Scalar,
        z_length: Scalar,
        border_radius: Scalar,
    },
    /// Constructs a collider with [`Collider::cylinder`].
    #[cfg(feature = "3d")]
    Cylinder { height: Scalar, radius: Scalar },
    /// Constructs a collider with [`Collider::cone`].
    #[cfg(feature = "3d")]
    Cone { height: Scalar, radius: Scalar },
    /// Constructs a collider with [`Collider::capsule`].
    Capsule { height: Scalar, radius: Scalar },
    /// Constructs a collider with [`Collider::capsule_endpoints`].
    CapsuleEndpoints {
        a: Vector,
        b: Vector,
        radius: Scalar,
    },
    /// Constructs a collider with [`Collider::half_space`].
    HalfSpace { outward_normal: Vector },
    /// Constructs a collider with [`Collider::segment`].
    Segment { a: Vector, b: Vector },
    /// Constructs a collider with [`Collider::triangle`].
    Triangle { a: Vector, b: Vector, c: Vector },
    /// Constructs a collider with [`Collider::regular_polygon`].
    #[cfg(feature = "2d")]
    RegularPolygon { circumradius: f32, sides: usize },
    /// Constructs a collider with [`Collider::polyline`].
    Polyline {
        vertices: Vec<Vector>,
        indices: Option<Vec<[u32; 2]>>,
    },
    /// Constructs a collider with [`Collider::trimesh`].
    Trimesh {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
    },
    /// Constructs a collider with [`Collider::trimesh_with_config`].
    TrimeshWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        flags: TrimeshFlags,
    },
    /// Constructs a collider with [`Collider::convex_decomposition`].
    #[cfg(feature = "2d")]
    ConvexDecomposition {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
    },
    /// Constructs a collider with [`Collider::convex_decomposition`].
    #[cfg(feature = "3d")]
    ConvexDecomposition {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
    },
    /// Constructs a collider with [`Collider::convex_decomposition_with_config`].
    #[cfg(feature = "2d")]
    ConvexDecompositionWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
        params: VhacdParameters,
    },
    /// Constructs a collider with [`Collider::convex_decomposition_with_config`].
    #[cfg(feature = "3d")]
    ConvexDecompositionWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        params: VhacdParameters,
    },
    /// Constructs a collider with [`Collider::convex_hull`].
    #[cfg(feature = "2d")]
    ConvexHull { points: Vec<Vector> },
    /// Constructs a collider with [`Collider::convex_hull`].
    #[cfg(feature = "3d")]
    ConvexHull { points: Vec<Vector> },
    /// Constructs a collider with [`Collider::heightfield`].
    #[cfg(feature = "2d")]
    Heightfield { heights: Vec<Scalar>, scale: Vector },
    /// Constructs a collider with [`Collider::heightfield`].
    #[cfg(feature = "3d")]
    Heightfield {
        heights: Vec<Vec<Scalar>>,
        scale: Vector,
    },
    /// Constructs a collider with [`Collider::trimesh_from_mesh`].
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[default]
    TrimeshFromMesh,
    /// Constructs a collider with [`Collider::trimesh_from_mesh_with_config`].
    #[cfg(all(
        feature = "3d",
        feature = "collider-from-mesh",
        feature = "default-collider"
    ))]
    TrimeshFromMeshWithConfig(TrimeshFlags),
    /// Constructs a collider with [`Collider::convex_decomposition_from_mesh`].
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    ConvexDecompositionFromMesh,
    /// Constructs a collider with [`Collider::convex_decomposition_from_mesh_with_config`].
    #[cfg(all(
        feature = "3d",
        feature = "collider-from-mesh",
        feature = "default-collider"
    ))]
    ConvexDecompositionFromMeshWithConfig(VhacdParameters),
    /// Constructs a collider with [`Collider::convex_hull_from_mesh`].
    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    ConvexHullFromMesh,
}

impl ColliderConstructor {
    /// Returns `true` if the collider type requires a mesh to be generated.
    pub fn requires_mesh(&self) -> bool {
        #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
        {
            matches!(
                self,
                Self::TrimeshFromMesh
                    | Self::TrimeshFromMeshWithConfig(_)
                    | Self::ConvexDecompositionFromMesh
                    | Self::ConvexDecompositionFromMeshWithConfig(_)
                    | Self::ConvexHullFromMesh
            )
        }
        #[cfg(not(all(feature = "3d", feature = "collider-from-mesh")))]
        {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::query::QueryData;
    #[cfg(feature = "bevy_scene")]
    use bevy::scene::ScenePlugin;

    #[test]
    fn collider_constructor_requires_no_mesh_on_primitive() {
        let mut app = create_test_app();

        let entity = app.world.spawn(PRIMITIVE_COLLIDER.clone()).id();

        app.update();

        assert!(app.query_ok::<&Collider>(entity));
        assert!(app.query_err::<&ColliderConstructor>(entity));
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[test]
    #[should_panic]
    fn collider_constructor_requires_mesh_on_computed() {
        let mut app = create_test_app();

        app.world.spawn(COMPUTED_COLLIDER.clone());

        app.update();
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[test]
    fn collider_constructor_converts_mesh_on_computed() {
        let mut app = create_test_app();

        let mesh_handle = app.add_mesh();
        let entity = app
            .world
            .spawn((COMPUTED_COLLIDER.clone(), mesh_handle))
            .id();

        app.update();

        assert!(app.query_ok::<&Collider>(entity));
        assert!(app.query_ok::<&Handle<Mesh>>(entity));
        assert!(app.query_err::<&ColliderConstructor>(entity));
    }

    #[test]
    fn collider_constructor_hierarchy_does_nothing_on_self_with_primitive() {
        let mut app = create_test_app();

        let entity = app
            .world
            .spawn(ColliderConstructorHierarchy::new(
                PRIMITIVE_COLLIDER.clone(),
            ))
            .id();

        app.update();

        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
        assert!(app.query_err::<&Collider>(entity));
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[test]
    fn collider_constructor_hierarchy_does_nothing_on_self_with_computed() {
        let mut app = create_test_app();

        let mesh_handle = app.add_mesh();
        let entity = app
            .world
            .spawn((
                ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()),
                mesh_handle,
            ))
            .id();

        app.update();

        assert!(app.query_ok::<&Handle<Mesh>>(entity));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
        assert!(app.query_err::<&Collider>(entity));
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[test]
    fn collider_constructor_hierarchy_does_not_require_mesh_on_self_with_computed() {
        let mut app = create_test_app();

        let entity = app
            .world
            .spawn(ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()))
            .id();

        app.update();

        assert!(app.query_err::<&Collider>(entity));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
    }

    #[test]
    fn collider_constructor_hierarchy_inserts_primitive_colliders_on_all_descendants() {
        let mut app = create_test_app();

        // Hierarchy:
        // - parent
        //   - child1
        //   - child2
        //     - child3

        let parent = app
            .world
            .spawn(ColliderConstructorHierarchy::new(
                PRIMITIVE_COLLIDER.clone(),
            ))
            .id();
        let child1 = app.world.spawn(()).id();
        let child2 = app.world.spawn(()).id();
        let child3 = app.world.spawn(()).id();

        app.world
            .entity_mut(parent)
            .push_children(&[child1, child2]);
        app.world.entity_mut(child2).push_children(&[child3]);

        app.update();

        // No entities should have ColliderConstructorHierarchy
        assert!(app.query_err::<&ColliderConstructorHierarchy>(parent));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child1));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child2));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child3));

        assert!(app.query_err::<&Collider>(parent));
        assert!(app.query_ok::<&Collider>(child1));
        assert!(app.query_ok::<&Collider>(child2));
        assert!(app.query_ok::<&Collider>(child3));
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    #[test]
    fn collider_constructor_hierarchy_inserts_computed_colliders_only_on_descendants_with_mesh() {
        let mut app = create_test_app();
        let mesh_handle = app.add_mesh();

        // Hierarchy:
        // - parent
        //   - child1 (no mesh)
        //   - child2 (no mesh)
        //     - child3 (mesh)
        //   - child4 (mesh)
        //     - child5 (no mesh)
        //   - child6 (mesh)
        //   - child7 (mesh)
        //     - child8 (mesh)

        let parent = app
            .world
            .spawn(ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()))
            .id();
        let child1 = app.world.spawn(()).id();
        let child2 = app.world.spawn(()).id();
        let child3 = app.world.spawn(mesh_handle.clone()).id();
        let child4 = app.world.spawn(mesh_handle.clone()).id();
        let child5 = app.world.spawn(()).id();
        let child6 = app.world.spawn(mesh_handle.clone()).id();
        let child7 = app.world.spawn(mesh_handle.clone()).id();
        let child8 = app.world.spawn(mesh_handle.clone()).id();

        app.world
            .entity_mut(parent)
            .push_children(&[child1, child2, child4, child6, child7]);
        app.world.entity_mut(child2).push_children(&[child3]);
        app.world.entity_mut(child4).push_children(&[child5]);
        app.world.entity_mut(child7).push_children(&[child8]);

        app.update();

        // No entities should have ColliderConstructorHierarchy
        assert!(app.query_err::<&ColliderConstructorHierarchy>(parent));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child1));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child2));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child3));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child4));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child5));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child6));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child7));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child8));

        assert!(app.query_err::<&Collider>(parent));
        assert!(app.query_err::<&Collider>(child1));
        assert!(app.query_err::<&Collider>(child2));
        assert!(app.query_ok::<&Collider>(child3));
        assert!(app.query_ok::<&Collider>(child4));
        assert!(app.query_err::<&Collider>(child5));
        assert!(app.query_ok::<&Collider>(child6));
        assert!(app.query_ok::<&Collider>(child7));
        assert!(app.query_ok::<&Collider>(child8));
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh", feature = "bevy_scene"))]
    #[test]
    fn collider_constructor_hierarchy_inserts_correct_configs_on_scene() {
        use parry::shape::ShapeType;

        let mut app = create_gltf_test_app();

        let scene_handle = app
            .world
            .resource_mut::<AssetServer>()
            .load("ferris.glb#Scene0");

        let hierarchy = app
            .world
            .spawn((
                SceneBundle {
                    scene: scene_handle,
                    ..default()
                },
                ColliderConstructorHierarchy::new(ColliderConstructor::ConvexDecompositionFromMesh)
                    // Use a primitive collider for the left arm.
                    .with_constructor_for_name("armL_mesh", PRIMITIVE_COLLIDER)
                    .with_density_for_name("armL_mesh", 2.0)
                    // Remove the right arm. Don't worry, crabs can regrow lost limbs!
                    .without_constructor_for_name("armR_mesh"),
                RigidBody::Dynamic,
            ))
            .id();

        while app
            .world
            .resource::<Events<bevy::scene::SceneInstanceReady>>()
            .is_empty()
        {
            app.update();
        }
        app.update();

        assert!(app.query_err::<&ColliderConstructorHierarchy>(hierarchy));
        assert!(app.query_err::<&Collider>(hierarchy));

        // Check densities
        let densities: HashMap<_, _> = app
            .world
            .query::<(&Name, &ColliderDensity)>()
            .iter(&app.world)
            .map(|(name, density)| (name.to_string(), density.0))
            .collect();

        assert_eq!(densities["eyes_mesh"], 1.0);
        assert_eq!(densities["armL_mesh"], 2.0);
        assert!(densities.get("armR_mesh").is_none());

        // Check collider shape types
        let colliders: HashMap<_, _> = app
            .world
            .query::<(&Name, &Collider)>()
            .iter(&app.world)
            .map(|(name, collider)| (name.to_string(), collider))
            .collect();

        assert_eq!(
            colliders["eyes_mesh"].shape().shape_type(),
            ShapeType::Compound
        );
        assert_eq!(
            colliders["armL_mesh"].shape().shape_type(),
            ShapeType::Capsule
        );
        assert!(colliders.get("armR_mesh").is_none());
    }

    const PRIMITIVE_COLLIDER: ColliderConstructor = ColliderConstructor::Capsule {
        height: 1.0,
        radius: 0.5,
    };

    #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
    const COMPUTED_COLLIDER: ColliderConstructor = ColliderConstructor::TrimeshFromMesh;

    fn create_test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            ScenePlugin,
            PhysicsPlugins::default(),
        ))
        .init_resource::<Assets<Mesh>>();

        app
    }

    #[cfg(all(feature = "3d", feature = "collider-from-mesh", feature = "bevy_scene"))]
    fn create_gltf_test_app() -> App {
        use bevy::{diagnostic::DiagnosticsPlugin, winit::WinitPlugin};

        let mut app = App::new();
        app.add_plugins((
            DefaultPlugins
                .build()
                .disable::<WinitPlugin>()
                .disable::<DiagnosticsPlugin>(),
            PhysicsPlugins::default(),
        ));
        app.finish();
        app.cleanup();
        app
    }

    trait AppExt {
        fn query_ok<D: QueryData>(&mut self, entity: Entity) -> bool;
        fn query_err<D: QueryData>(&mut self, entity: Entity) -> bool {
            !self.query_ok::<D>(entity)
        }

        #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
        fn add_mesh(&mut self) -> Handle<Mesh>;
    }

    impl AppExt for App {
        fn query_ok<D: QueryData>(&mut self, entity: Entity) -> bool {
            let mut query = self.world.query::<D>();
            let component = query.get(&self.world, entity);
            component.is_ok()
        }

        #[cfg(all(feature = "3d", feature = "collider-from-mesh"))]
        fn add_mesh(&mut self) -> Handle<Mesh> {
            self.world
                .get_resource_mut::<Assets<Mesh>>()
                .unwrap()
                .add(Mesh::from(Cuboid::default()))
        }
    }
}
