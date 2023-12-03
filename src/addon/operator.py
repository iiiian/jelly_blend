import bpy
from . import helper
from . import jellyblend_engine as engine
from .simulation_state import SimState


class JBCreateCollection(bpy.types.Operator):
    bl_idname = "jb_operators.jb_create_jb_collection"
    bl_label = "Create Jelly Blend World"
    bl_description = "Create a collection called Jelly Blend World for simuation"

    @classmethod
    def poll(csl, context):
        return True

    def execute(self, context):
        helper.get_jb_collection(context)
        return {"FINISHED"}


class JBSimulate(bpy.types.Operator):
    bl_idname = "jb_operators.jb_simulate"
    bl_label = "Start soft body simulation"
    bl_description = "Start Jelly Blend soft body simulation"

    timer = None
    phy_world = None

    is_simulating = False
    is_inserting_shapekey = False
    frame_start = 0
    frame_end = 0
    current_frame = 0

    def __init__(self):
        SimState.is_running = True

    def __del__(self):
        SimState.is_running = False

    @classmethod
    def poll(csl, context):
        if helper.does_jb_collection_exist(context):
            return not SimState.is_running
        return False

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        if self.is_inserting_shapekey:
            return {"RUNNING_MODAL"}

        if self.is_simulating:
            return {"RUNNING_MODAL"}

        if self.current_frame == self.frame_end:
            print(
                f"sim fin, current frame {self.current_frame}, frame_end {self.frame_end}",
            )
            return {"FINISHED"}

        if SimState.is_interrupted:
            SimState.is_interrupted = False
            return {"FINISHED"}

        # simulate
        self.is_simulating = True
        try:
            helper.jb_engine_exp_handler(self.phy_world.next_frame)()
        except helper.JBEngineException as err:
            self.report(
                {"ERROR"},
                str(err),
            )
            return {"FINISHED"}
        self.is_simulating = False

        # update status
        self.current_frame += 1
        SimState.finished_frame += 1
        # inserted shapekeys
        self.is_inserting_shapekey = True
        try:
            helper.jb_engine_exp_handler(self.phy_world.insert_softbody_shapekey)()
        except helper.JBEngineException as err:
            self.report(
                {"ERROR"},
                str(err),
            )
            return {"FINISHED"}
        self.is_inserting_shapekey = False
        return {"RUNNING_MODAL"}

    def invoke(self, context, event):
        jb_world = helper.get_jb_collection(context)
        sim_settings = jb_world.jb_sim_setting

        scene = context.scene
        scene_frame_start = scene.frame_start
        scene_frame_end = scene.frame_end

        self.frame_start = sim_settings.frame_start
        self.frame_end = sim_settings.frame_end
        self.current_frame = self.frame_start

        SimState.finished_frame = 0
        SimState.total_frame = self.frame_end - self.frame_start

        if self.frame_start < scene_frame_start or self.frame_end > scene_frame_end:
            self.report(
                {"ERROR"},
                "frame execeed blender frame limit",
            )
            return {"CANCELLED"}

        if sim_settings.frame_start >= sim_settings.frame_end:
            self.report(
                {"ERROR"},
                "frame start >= frame end",
            )
            return {"CANCELLED"}

        colliders = []
        softbodies = []

        for obj in jb_world.all_objects:
            if obj.jb_property.object_type == "TYPE_COLLIDER":
                colliders.append(obj)
            if obj.jb_property.object_type == "TYPE_SOFTBODY":
                softbodies.append(obj)

        if len(softbodies) == 0:
            return {"CANCELLED"}

        for collider in colliders:
            mesh = collider.to_mesh()
            if not helper.has_triangular_mesh(mesh):
                self.report(
                    {"ERROR"},
                    "collider " + collider.name + " has non triangular mesh",
                )
                collider.to_mesh_clear()
                return {"CANCELLED"}
            collider.to_mesh_clear()

        for soft in softbodies:
            if not soft.jb_property.softbody.has_internal_mesh:
                self.report(
                    {"ERROR"},
                    "soft body "
                    + soft.name
                    + " has no softbody mesh, please generate them is physics panel",
                )
                return {"CANCELLED"}

            jb_vert_num = soft["jb_softbody_mesh_surface_vertex_num"]
            mesh_vert_num = len(soft.data.vertices)
            if jb_vert_num != mesh_vert_num:
                self.report(
                    {"ERROR"},
                    "you seem to have modified the mesh of "
                    + soft.name
                    + ", please regenerate softbody mesh is physics panel",
                )
                return {"CANCELLED"}

        # load sim settings
        self.phy_world = engine.PhysicsWorld()
        phy_setting = engine.PhysicsWorldSetting()

        phy_setting.solver_substep_num = sim_settings.solver_frame_substep
        phy_setting.frame_substep_num = sim_settings.collision_frame_substep
        phy_setting.frame_rate = sim_settings.frame_rate
        phy_setting.spatial_map_size_multiplier = sim_settings.colli_map_size
        phy_setting.manual_spatial_cell_size = sim_settings.manual_spatial_cell_size
        phy_setting.spatial_cell_size = sim_settings.spatial_cell_size
        phy_setting.manual_passive_collision_distance = (
            sim_settings.manual_passive_collision_distance
        )
        phy_setting.passive_collision_distance = sim_settings.passive_collision_distance

        self.phy_world.load_setting(phy_setting)

        # load colliders
        for collider in colliders:
            self.phy_world.add_fixedbody(collider)

        # load softbodies
        for soft in softbodies:
            soft_setting = engine.SoftBodySetting()
            soft_props = soft.jb_property.softbody

            soft_setting.density = soft_props.density
            soft_setting.gravity = 9.8
            soft_setting.youngs_modulus = soft_props.stiffness * 1e7
            soft_setting.poissons_ratio = 0.5 - soft_props.compressibility * 0.01
            soft_setting.damping = soft_props.damping
            soft_setting.friction = soft_props.friction
            soft_setting.detect_self_collision = soft_props.self_collision

            self.phy_world.add_softbody(soft, soft_setting)

        try:
            helper.jb_engine_exp_handler(self.phy_world.prepare_simulation)(
                sim_settings.frame_start, False
            )
        except helper.JBEngineException as err:
            self.report(
                {"ERROR"},
                str(err),
            )
            return {"CANCELLED"}

        context.window_manager.modal_handler_add(self)
        self.timer = context.window_manager.event_timer_add(0.01, window=context.window)

        self.phy_world.dump_to_file("/home/ian/local_code/jelly_blend/test/world_data")

        return {"RUNNING_MODAL"}


class JBStopSimulation(bpy.types.Operator):
    bl_idname = "jb_operators.jb_stop_simulation"
    bl_label = "Stop soft body simulation"
    bl_description = "Stop Jelly Blend soft body simulation"

    @classmethod
    def poll(csl, context):
        return SimState.is_running

    def execute(self, context):
        SimState.is_interrupted = True
        self.report(
            {"INFO"},
            "Stopping simulation, please wait",
        )
        return {"FINISHED"}


class JBCleanSimulation(bpy.types.Operator):
    bl_idname = "jb_operators.jb_clean_simulation"
    bl_label = "Clean soft body simulation"
    bl_description = "Clean Jelly Blend soft body simulation"

    @classmethod
    def poll(csl, context):
        return helper.does_jb_collection_exist(context) and not SimState.is_running

    def execute(self, context):
        jb_world = helper.get_jb_collection(context)

        softbodies = []
        for obj in jb_world.all_objects:
            if obj.jb_property.object_type == "TYPE_SOFTBODY":
                softbodies.append(obj)

        if len(softbodies) == 0:
            return {"CANCELLED"}

        for soft in softbodies:
            soft.shape_key_clear()

        return {"FINISHED"}


class JBAdd(bpy.types.Operator):
    bl_idname = "jb_operators.jb_add"
    bl_label = "Add Jelly Blend object"
    bl_description = "Add active object as Jelly Blend object"

    @classmethod
    def poll(csl, context):
        obj = bpy.context.active_object
        return helper.is_mesh_obj(obj)

    def execute(self, context):
        obj = bpy.context.active_object
        obj.jb_property.is_active = True
        helper.add_to_jb_collection(obj, context)
        return {"FINISHED"}


class JBAddSoftBody(bpy.types.Operator):
    bl_idname = "jb_operators.jb_add_softbody"
    bl_label = "Add Jelly Blend soft body"
    bl_description = "Add active object as Jelly Blend soft body"

    @classmethod
    def poll(csl, context):
        obj = bpy.context.active_object
        return helper.is_mesh_obj(obj)

    def execute(self, context):
        obj = bpy.context.active_object
        obj.jb_property.is_active = True
        obj.jb_property.object_type = "TYPE_SOFTBODY"
        helper.add_to_jb_collection(obj, context)
        return {"FINISHED"}


class JBAddCollider(bpy.types.Operator):
    bl_idname = "jb_operators.jb_add_collider"
    bl_label = "Add Jelly Blend collider"
    bl_description = "Add active object as Jelly Blend collider"

    @classmethod
    def poll(csl, context):
        obj = bpy.context.active_object
        return helper.is_mesh_obj(obj)

    def execute(self, context):
        obj = bpy.context.active_object
        obj.jb_property.is_active = True
        obj.jb_property.object_type = "TYPE_COLLIDER"
        helper.add_to_jb_collection(obj, context)
        return {"FINISHED"}


class JBRemove(bpy.types.Operator):
    bl_idname = "jb_operators.jb_remove"
    bl_label = "Remove Jelly Blend object"
    bl_description = "Remove active object from Jelly Blend object"

    @classmethod
    def poll(csl, context):
        obj = bpy.context.active_object
        return obj.jb_property.is_active

    def execute(self, context):
        obj = bpy.context.active_object

        if obj.jb_property.softbody.has_internal_mesh:
            helper.delete_softbody_mesh(obj)

        obj.jb_property.is_active = False
        obj.jb_property.object_type = "TYPE_NONE"
        helper.remove_from_jb_collection(obj, context)
        return {"FINISHED"}


class JBGenerateSoftBodyMesh(bpy.types.Operator):
    bl_idname = "jb_operators.jb_generate_softbody_mesh"
    bl_label = "Generate Jelly Blend soft body mesh"
    bl_description = "Generate the internal tetrahedron mesh of Jelly Blend soft body"

    @classmethod
    def poll(csl, context):
        obj_props = bpy.context.active_object.jb_property
        return obj_props.is_active and obj_props.object_type == "TYPE_SOFTBODY"

    def execute(self, context):
        depsgraph = context.evaluated_depsgraph_get()
        obj = bpy.context.active_object
        bpy.ops.object.transform_apply()
        object_eval = obj.evaluated_get(depsgraph)
        mesh = object_eval.to_mesh()
        mesh.transform(object_eval.matrix_world)

        if not helper.has_triangular_mesh(mesh):
            self.report(
                {"ERROR"},
                "Object has non triangular mesh, can't generate softbody mesh. Please use triangulate modifier",
            )
            return {"CANCELLED"}

        soft_props = obj.jb_property.softbody
        if soft_props.enable_volume_constrain and soft_props.max_volume > 1e-4:
            max_vol = soft_props.max_volume
        else:
            max_vol = -1

        try:
            geometry = helper.jb_engine_exp_handler(engine.softbody_mesh_from_bl_mesh)(
                mesh, max_volume=max_vol
            )
        except helper.JBEngineException as err:
            self.report(
                {"ERROR"},
                str(err),
            )
            return {"CANCELLED"}
        finally:
            object_eval.to_mesh_clear()

        obj["jb_softbody_mesh_surface_vertex_num"] = geometry.surface_vertex_num
        obj["jb_softbody_mesh_initial_vertices"] = geometry.all_vertices
        obj["jb_softbody_mesh_surface_faces"] = geometry.surface_faces
        obj["jb_softbody_mesh_edges"] = geometry.all_edges
        obj["jb_softbody_mesh_surface_edges"] = geometry.surface_edges
        obj["jb_softbody_mesh_tetrahedra"] = geometry.tetrahedra

        soft_prop = obj.jb_property.softbody
        soft_prop.total_vertex_count = len(geometry.all_vertices)
        soft_prop.total_edge_count = len(geometry.all_edges)
        soft_prop.tetrahedron_count = len(geometry.tetrahedra)
        soft_prop.has_internal_mesh = True

        self.report({"INFO"}, "successfully generate softbody mesh")
        return {"FINISHED"}


class JBDeleteSoftBodyMesh(bpy.types.Operator):
    bl_idname = "jb_operators.jb_delete_softbody_mesh"
    bl_label = "Delete Jelly Blend soft body mesh"
    bl_description = "Delete the internal tetrahedron mesh of Jelly Blend soft body"

    @classmethod
    def poll(csl, context):
        obj_props = bpy.context.active_object.jb_property
        return obj_props.is_active and obj_props.softbody.has_internal_mesh

    def execute(self, context):
        obj = bpy.context.active_object
        helper.delete_softbody_mesh(obj)

        self.report({"INFO"}, "successfully delete softbody mesh")
        return {"FINISHED"}


def register():
    bpy.utils.register_class(JBCreateCollection)
    bpy.utils.register_class(JBSimulate)
    bpy.utils.register_class(JBStopSimulation)
    bpy.utils.register_class(JBCleanSimulation)
    bpy.utils.register_class(JBAdd)
    bpy.utils.register_class(JBAddSoftBody)
    bpy.utils.register_class(JBAddCollider)
    bpy.utils.register_class(JBRemove)
    bpy.utils.register_class(JBGenerateSoftBodyMesh)
    bpy.utils.register_class(JBDeleteSoftBodyMesh)


def unregister():
    bpy.utils.unregister_class(JBCreateCollection)
    bpy.utils.unregister_class(JBSimulate)
    bpy.utils.unregister_class(JBStopSimulation)
    bpy.utils.unregister_class(JBCleanSimulation)
    bpy.utils.unregister_class(JBAdd)
    bpy.utils.unregister_class(JBAddSoftBody)
    bpy.utils.unregister_class(JBAddCollider)
    bpy.utils.unregister_class(JBRemove)
    bpy.utils.unregister_class(JBGenerateSoftBodyMesh)
    bpy.utils.unregister_class(JBDeleteSoftBodyMesh)
