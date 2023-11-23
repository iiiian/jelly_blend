from .jellyblend_engine import SoftBodyMesh
import bpy


def add_shapekey_range(name: str, frame_start: int, frame_end: int):
    obj = bpy.data.objects.get(name)
    assert not obj is None

    shapekeys = obj.data.shape_keys
    if shapekeys is None:
        obj.shape_key_add(name="Base", from_mix=False)
        shapekeys = obj.data.shape_keys

    for frame in range(frame_start, frame_end + 1):
        frame_name = "frame" + str(frame)
        shapekey = shapekeys.key_blocks.get(frame_name)
        if shapekey is None:
            obj.shape_key_add(name=frame_name, from_mix=False)

    shapekeys.use_relative = False


def insert_softbody_shapekeys(softbody_mesh: SoftBodyMesh, frame: int):
    name = softbody_mesh.bl_object_name
    obj = bpy.data.objects.get(name)
    assert not obj is None

    surface_vertex_num = softbody_mesh.surface_vertex_num
    if len(obj.data.vertices) != surface_vertex_num:
        raise RuntimeError("obj mesh is modified")

    frame_name = "frame" + str(frame)
    shapekeys = obj.data.shape_keys
    assert not shapekeys is None

    shapekey = shapekeys.key_blocks.get(frame_name)
    assert not shapekey is None

    for i in range(surface_vertex_num):
        shape_key_point = shapekey.data[i]
        assert not shape_key_point is None
        for j in range(3):
            shape_key_point.co[j] = softbody_mesh.vertices[3 * i + j]

    # insert animation
    shapekeys.eval_time = shapekey.frame
    shapekeys.keyframe_insert(data_path="eval_time", frame=frame)


def insert_all_softbodies_shapekeys(softbody_meshes: list[SoftBodyMesh], frame: int):
    for mesh in softbody_meshes:
        insert_softbody_shapekeys(mesh, frame)
