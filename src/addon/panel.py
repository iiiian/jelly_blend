import bpy
from . import helper


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


class JB_PT_NoneTypePanel(bpy.types.Panel):
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


class JB_PT_ColliderTypePanel(bpy.types.Panel):
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


class JB_PT_SoftBodyTypePanel(bpy.types.Panel):
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


class JB_PT_SimPanel(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_category = "Jelly Blend"
    bl_region_type = "UI"
    bl_label = "Simulation"

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

            box.label(text="Start")
            column = box.column()
            column.prop(sim_settings, "frame_start")
            column.prop(sim_settings, "frame_end")
            column.separator()
            column.operator("jb_operators.jb_simulate", text="Simualate")
            column.operator("jb_operators.jb_clean_simulation", text="Clean Simulation")

            box = self.layout.box()
            box.label(text="Basic Settings")
            column = box.column()
            column.prop(sim_settings, "frame_rate")
            column.prop(sim_settings, "collision_frame_substep", text="frame substep")
            column.prop(sim_settings, "solver_frame_substep")

            box = self.layout.box()
            box.label(text="Advanced Settings")
            column = box.column()
            column.prop(sim_settings, "passive_collision_distance")
            column.prop(sim_settings, "colli_map_size")

        """
        sim_setting = bpy.context.scene.jb_sim_setting
        column.prop(sim_setting, "solver_frame_substep")
        column.prop(sim_setting, "collision_frame_substep")
        column.prop(sim_setting, "max_collision_distance")
        """


def register():
    bpy.types.PHYSICS_PT_add.append(append_to_PHYSICS_PT_add_panel)
    bpy.utils.register_class(JB_PT_NoneTypePanel)
    bpy.utils.register_class(JB_PT_ColliderTypePanel)
    bpy.utils.register_class(JB_PT_SoftBodyTypePanel)
    bpy.utils.register_class(JB_PT_SimPanel)


def unregister():
    bpy.types.PHYSICS_PT_add.remove(append_to_PHYSICS_PT_add_panel)
    bpy.utils.unregister_class(JB_PT_NoneTypePanel)
    bpy.utils.unregister_class(JB_PT_ColliderTypePanel)
    bpy.utils.unregister_class(JB_PT_SoftBodyTypePanel)
    bpy.utils.unregister_class(JB_PT_SimPanel)
