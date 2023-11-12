bl_info = {
    "name": "Jelly Blend Soft Body",
    "author": "Tzu Hiang Su",
    "description": "A softbody simulation addon",
    "blender": (3, 60, 0),
    "category": "Animation",
}

import bpy
from . import property, operator, panel

def register():
    property.register()
    operator.register()
    panel.register()
    print("finished loading jelly blend")


def unregister():
    property.unregister()
    operator.unregister()
    panel.unregister()
    print("finished unloading jelly blend")
