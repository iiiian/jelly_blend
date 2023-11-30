import bpy
from . import helper
from .simulation_state import SimState


def append_to_PHYSICS_PT_add_panel(self, context):
    obj = bpy.context.active_object
    # if not (obj.type == "MESH" or obj.type == "EMPTY", obj.type == "CURVE"):
    #     return

    if obj is None:
        return
    # excludes object can not be converted to mesh
    if obj.type not in {"MESH", "CURVE", "SURFACE", "FONT", "META"}:
        return

    column = self.layout.column(align=True)
    split = column.split(factor=0.5)
    column_left = split.column()
    column_right = split.column()

    if obj.jb_property.is_active:
        column_right.operator("jb_operators.jb_remove", text="JB Soft Body", icon="X")
    else:
        column_right.operator(
            "jb_operators.jb_add", text="JB Soft Body", icon="MOD_SOFT"
        )


def make_twin_label(
    layout: bpy.types.UILayout, factor: float, text_left: str, text_right: str
):
    split = layout.split(factor=0.5)
    column_left = split.column()
    column_left.label(text=text_left)
    column_right = split.column()
    column_right.label(text=text_right)


class JB_PT_NoneType(bpy.types.Panel):
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_label = "JB Soft Body"

    @classmethod
    def poll(cls, context):
        obj_props = bpy.context.active_object.jb_property
        return obj_props.is_active and obj_props.object_type == "TYPE_NONE"

    def draw(self, context):
        obj = bpy.context.active_object
        obj_props = obj.jb_property

        column = self.layout.column()
        column.prop(obj_props, "object_type")


class JB_PT_ColliderType(bpy.types.Panel):
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_label = "JB Collider"

    @classmethod
    def poll(cls, context):
        obj_props = bpy.context.active_object.jb_property
        return obj_props.is_active and obj_props.object_type == "TYPE_COLLIDER"

    def draw(self, context):
        obj = bpy.context.active_object
        obj_props = obj.jb_property

        # obejct type selector
        column = self.layout.column()
        column.prop(obj_props, "object_type")

        # collider panel
        # box = self.layout.box()
        # box.label(text="Collider")


class JB_PT_SoftBodyType(bpy.types.Panel):
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    bl_label = "JB Soft Body"

    @classmethod
    def poll(cls, context):
        obj_props = bpy.context.active_object.jb_property
        return obj_props.is_active and obj_props.object_type == "TYPE_SOFTBODY"

    def draw(self, context):
        obj = bpy.context.active_object
        obj_props = obj.jb_property
        softbody_props = obj_props.softbody

        # obejct type selector
        column = self.layout.column()
        column.prop(obj_props, "object_type")

        # softbody mesh panel
        box = self.layout.box()
        box.label(text="Soft Body Mesh")
        column = box.column()

        if softbody_props.has_internal_mesh:
            make_twin_label(
                column, 0.5, "vertex count:", str(softbody_props.total_vertex_count)
            )
            make_twin_label(
                column, 0.5, "edge count:", str(softbody_props.total_edge_count)
            )
            make_twin_label(
                column, 0.5, "tetrahedron count:", str(softbody_props.tetrahedron_count)
            )
        else:
            column.label(text="no softbody mesh", icon="ERROR")
            column.label(text="TIPS: non triangular mesh is not allowed")

        column.prop(
            softbody_props,
            "enable_volume_constrain",
            text="enable volume constrain",
        )
        if softbody_props.enable_volume_constrain:
            column.prop(softbody_props, "max_volume", text="max_volume")

        if softbody_props.has_internal_mesh:
            column.operator(
                "jb_operators.jb_generate_softbody_mesh", text="regenerate mesh"
            )
            column.operator("jb_operators.jb_delete_softbody_mesh", text="delete mesh")
        else:
            column.operator(
                "jb_operators.jb_generate_softbody_mesh", text="generate mesh"
            )

        # softbody properties panel
        box = self.layout.box()
        box.label(text="Soft Body Properties")

        column = box.column()
        column.prop(softbody_props, "density")
        column.prop(softbody_props, "stiffness")
        column.prop(softbody_props, "compressibility")
        column.prop(softbody_props, "damping")
        column.prop(softbody_props, "friction")
        box.prop(softbody_props, "self_collision")


class JB_PT_SimControl(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_category = "Jelly Blend"
    bl_region_type = "UI"
    bl_label = "Simulation Control"

    @classmethod
    def poll(cls, context):
        return True

    def draw(self, context):
        box = self.layout.box()

        if not helper.does_jb_collection_exist(context):
            box.label(text="No simulation world", icon="ERROR")
            box.operator("jb_operators.jb_create_jb_collection", text="Create")
        else:
            jb_world = helper.get_jb_collection(context)
            sim_settings = jb_world.jb_sim_setting

            column = box.column()
            column.prop(sim_settings, "frame_start")
            column.prop(sim_settings, "frame_end")
            column.separator()
            if SimState.is_running:
                column.progress(
                    text="Baking...",
                    factor=SimState.finished_frame / SimState.total_frame,
                )
                column.operator("jb_operators.jb_stop_simulation", text="Stop")
            else:
                column.operator("jb_operators.jb_simulate", text="Simualate")
            column.operator("jb_operators.jb_clean_simulation", text="Clean Simulation")


class JB_PT_SimTimeSteps(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_category = "Jelly Blend"
    bl_region_type = "UI"
    bl_label = "Time steps"

    @classmethod
    def poll(cls, context):
        return helper.does_jb_collection_exist(context)

    def draw(self, context):
        jb_world = helper.get_jb_collection(context)
        sim_settings = jb_world.jb_sim_setting

        box = self.layout.box()
        column = box.column()
        column.prop(sim_settings, "frame_rate")
        column.prop(sim_settings, "collision_frame_substep", text="frame substep")
        column.prop(sim_settings, "solver_frame_substep")


class JB_PT_SimCollisionSettings(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_category = "Jelly Blend"
    bl_region_type = "UI"
    bl_label = "Collision Settings"

    @classmethod
    def poll(cls, context):
        return helper.does_jb_collection_exist(context)

    def draw(self, context):
        jb_world = helper.get_jb_collection(context)
        sim_settings = jb_world.jb_sim_setting

        box = self.layout.box()
        column = box.column()
        column.prop(
            sim_settings,
            "manual_passive_collision_distance",
            text="Manual collsion distance",
        )
        if sim_settings.manual_passive_collision_distance:
            column.prop(sim_settings, "passive_collision_distance")

        box = self.layout.box()
        column = box.column()
        column.prop(sim_settings, "colli_map_size")


def register():
    bpy.types.PHYSICS_PT_add.append(append_to_PHYSICS_PT_add_panel)
    bpy.utils.register_class(JB_PT_NoneType)
    bpy.utils.register_class(JB_PT_ColliderType)
    bpy.utils.register_class(JB_PT_SoftBodyType)
    bpy.utils.register_class(JB_PT_SimControl)
    bpy.utils.register_class(JB_PT_SimTimeSteps)
    bpy.utils.register_class(JB_PT_SimCollisionSettings)


def unregister():
    bpy.types.PHYSICS_PT_add.remove(append_to_PHYSICS_PT_add_panel)
    bpy.utils.unregister_class(JB_PT_NoneType)
    bpy.utils.unregister_class(JB_PT_ColliderType)
    bpy.utils.unregister_class(JB_PT_SoftBodyType)
    bpy.utils.unregister_class(JB_PT_SimControl)
    bpy.utils.unregister_class(JB_PT_SimTimeSteps)
    bpy.utils.unregister_class(JB_PT_SimCollisionSettings)
