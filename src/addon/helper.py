import bpy
from . import jellyblend_engine as engine
from functools import wraps


def get_jb_collection(context):
    collection = bpy.data.collections.get("JB Soft Body World")
    if collection is None:
        collection = bpy.data.collections.new("JB Soft Body World")
        context.scene.collection.children.link(collection)
    return collection


def does_jb_collection_exist(context):
    collection = bpy.data.collections.get("JB Soft Body World")
    return not collection is None


def add_to_jb_collection(obj, context):
    if context is None:
        context = bpy.context
    jb_collection = get_jb_collection(context)
    if jb_collection.objects.get(obj.name):
        return
    jb_collection.objects.link(obj)


def remove_from_jb_collection(obj, context):
    if context is None:
        context = bpy.context
    jb_collection = get_jb_collection(context)

    num_collections = 0
    for collection in bpy.data.collections:
        if collection.name.startswith("RigidBodyWorld"):
            # The RigidBodyWorld collection (for RBD objects) is more hidden within the Blend file
            # and may not be apparent to many users. Ignore this collection in the count so that
            # it does not appear that the objects dissappear.
            continue
        if collection.objects.get(obj.name):
            num_collections += 1
    # link obj to scene to prevent obj from disappearing
    if num_collections == 1 and context.scene.collection.objects.get(obj.name) is None:
        context.scene.collection.objects.link(obj)

    if jb_collection.objects.get(obj.name):
        jb_collection.objects.unlink(obj)


def is_mesh_obj(obj):
    if obj is None:
        return False
    if obj.type not in {"MESH", "CURVE", "SURFACE", "FONT", "META"}:
        return False
    return True


def has_triangular_mesh(mesh):
    for poly in mesh.polygons:
        if poly.loop_total != 3:
            return False

    return True


def delete_softbody_mesh(obj):
    element_name = [
        "jb_softbody_mesh_surface_vertex_num",
        "jb_softbody_mesh_initial_vertices",
        "jb_softbody_mesh_surface_faces",
        "jb_softbody_mesh_edges",
        "jb_softbody_mesh_surface_edges",
        "jb_softbody_mesh_tetrahedra",
    ]

    for name in element_name:
        if hasattr(obj, name):
            del obj[name]

    obj.jb_property.softbody.has_internal_mesh = False


class JBEngineException(Exception):
    pass


def jb_engine_exp_handler(func):
    @wraps(func)
    def _handler(*args, **kwargs):
        try:
            ans = func(*args, **kwargs)
        except engine.MeshGenExceedIntMax:
            raise JBEngineException(
                "Object has too many faces, can't generate softbody mesh."
            )
        except engine.MeshGenOutOfMem:
            raise JBEngineException(
                "Running out of memory, can't generate softbody mesh."
            )
        except engine.MeshGenKnowBug:
            raise JBEngineException("bug occurs..., can't generate softbody mesh.")
        except engine.MeshGenSelfIntersection:
            raise JBEngineException(
                "detect self intersections, can't generate softbody mesh. Please use the built-in 3D Print ToolBox to detect mesh problems"
            )
        except engine.MeshGenSmallFeature:
            raise JBEngineException(
                "detect very small faces, can't generate softbody mesh. Please use the built-in 3D Print ToolBox to detect mesh problems"
            )
        except engine.MeshGenUnknown:
            raise JBEngineException("bug occurs..., can't generate softbody mesh.")
        except engine.BlMeshModified:
            raise JBEngineException(
                "you seem to have modified the mesh of softbody/collider, simulation stop"
            )
        except engine.BlObjectMissing:
            raise JBEngineException(
                "can't find softbody/collider object, might be deleted/renamed, simulation stop"
            )
        except engine.SimBlowUp:
            raise JBEngineException("softbody has blown up, simulation stop")
        else:
            return ans

    return _handler
