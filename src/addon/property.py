import bpy

jb_types = (
    ("TYPE_NONE", "None", "", 0),
    ("TYPE_SOFTBODY", "Soft Body", "can collide and deform", 1),
    ("TYPE_COLLIDER", "Collider", "can collide", 2),
)


class JBSoftBodyProperty(bpy.types.PropertyGroup):
    # simulation related
    density: bpy.props.FloatProperty(
        name="density", min=1e-5, default=1, description="the density of the soft body"
    )
    stiffness: bpy.props.FloatProperty(
        name="stiffness",
        min=1e-6,
        default=1,
        description="the stiffness of the soft body, affects bounciness. higher stiffness will require higher simulation substeps",
    )
    compressibility: bpy.props.FloatProperty(
        name="compressibility",
        min=0,
        max=49.99,
        default=0,
        description="the compressibility of the soft body, affects volume preservation",
    )
    damping: bpy.props.FloatProperty(
        name="damping",
        min=0,
        max=1,
        default=0,
        description="the damping of the soft body, affects the decay of vibration",
    )
    friction: bpy.props.FloatProperty(
        name="friction",
        min=0,
        max=1,
        default=0,
        description="the friction of the soft body, affects collisions",
    )
    self_collision: bpy.props.BoolProperty(name="self collision", default=False)

    # mesh related
    # mesh status
    has_internal_mesh: bpy.props.BoolProperty(name="has internal mesh", default=False)
    total_vertex_count: bpy.props.IntProperty(name="total vertex count", default=0)
    total_edge_count: bpy.props.IntProperty(name="total edge count", default=0)
    tetrahedron_count: bpy.props.IntProperty(name="tetrahedron count", default=0)
    # mesh generation settings
    enable_volume_constrain: bpy.props.BoolProperty(
        name="enable volume constrain", default=False
    )
    max_volume: bpy.props.FloatProperty(
        name="max volume",
        min=0,
        default=1,
        description="max tetrahedron volume during mesh generation, in m^3",
    )


class JBProperty(bpy.types.PropertyGroup):
    is_active: bpy.props.BoolProperty(name="is active", default=False)
    object_type: bpy.props.EnumProperty(name="Object Type", items=jb_types, default=0)
    softbody: bpy.props.PointerProperty(type=JBSoftBodyProperty)


class JBSimulationSetting(bpy.types.PropertyGroup):
    frame_start: bpy.props.IntProperty(name="frame start", min=0, default=1)
    frame_end: bpy.props.IntProperty(name="frame end", min=1, default=2)
    solver_frame_substep: bpy.props.IntProperty(
        name="solver frame substep",
        min=1,
        default=1,
        description="higher substep increase the quality of collision",
    )
    collision_frame_substep: bpy.props.IntProperty(
        name="collision frame substep",
        min=1,
        default=1,
        description="higher substep increase the quality of both collision and soft body physics",
    )
    frame_rate: bpy.props.IntProperty(name="frame rate", min=1, default=24)
    manual_passive_collision_distance: bpy.props.BoolProperty(
        name="manual passive collision distance",
        default=False,
        description="set passive collision distance manually",
    )
    passive_collision_distance: bpy.props.FloatProperty(
        name="passive collision distance",
        min=0,
        default=1e-3,
        description="the distance that triggers passive collision, in meter",
    )
    colli_map_size: bpy.props.IntProperty(
        name="collision map size",
        min=1,
        max=100,
        default=50,
        description="the size of the collision hash map, might speed up collision detection",
    )


def register():
    bpy.utils.register_class(JBSoftBodyProperty)
    bpy.utils.register_class(JBProperty)
    bpy.utils.register_class(JBSimulationSetting)

    bpy.types.Collection.jb_sim_setting = bpy.props.PointerProperty(
        type=JBSimulationSetting
    )
    bpy.types.Object.jb_property = bpy.props.PointerProperty(type=JBProperty)


def unregister():
    bpy.utils.unregister_class(JBSoftBodyProperty)
    bpy.utils.unregister_class(JBProperty)
    bpy.utils.unregister_class(JBSimulationSetting)

    del bpy.types.Collection.jb_sim_setting
    del bpy.types.Object.jb_property
