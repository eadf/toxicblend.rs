#!/usr/bin/python
import bpy
import bmesh
import mathutils
import math
from bpy.props import (
    BoolProperty,
    FloatProperty,
    PointerProperty,
)

bl_info = {
    "name": "Toxicblend - Metavolume (offline plugin)",
    'description': 'Generates volume from a lattice of edges using metaballs (offline plugin)',
    'author': 'EADF',
    "version": (0, 0, 13),
    'blender': (2, 92, 0),
    "category": "Object",
}


def get_rotation_to(from_vn, to_vn):
    """Returns the rotation quaternion needed when rotating fromVn to toVn. fromVn and toVn should be normalized."""
    if from_vn[0] == to_vn[0] and from_vn[1] == to_vn[1] and from_vn[2] == to_vn[2]:
        return mathutils.Quaternion((1.0, 0, 0, 0))
    cross_product = from_vn.cross(to_vn)
    cross_product.normalize()
    angle = math.acos(from_vn.dot(to_vn))
    return mathutils.Quaternion(cross_product, angle)


class ToxicBlend_MetaVolume(bpy.types.Operator):
    """Volumetric edge fill using meta capsules"""
    bl_idname = "object.toxicblend_metavolume"
    bl_label = "Toxicblend Metavolume volume"
    bl_description = 'Generates volume from a lattice of edges using metaballs (offline plugin)'
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    CAPSULE_VECTOR = mathutils.Vector((1.0, 0.0, 0.0))  # capsule orientation

    radius: FloatProperty(name="Radius", default=1.0, min=0.0001, max=1000,
                          description="Radius of the meta capsules")
    resolution: FloatProperty(name="Resolution", default=0.25, min=0.05, max=1,
                              description="Resolution of the meta capsules")
    threshold: FloatProperty(name="Threshold", default=0.05, min=0.001, max=1.99999,
                             description="Threshold of the meta capsules")

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def new_capsule(self, meta_factory, v0, v1, radius):
        segment = v1 - v0
        capsule = meta_factory.new()
        capsule.co = (v1 + v0) / 2.0
        capsule.type = 'CAPSULE'
        capsule.radius = radius
        capsule.size_x = segment.length / 2.0
        direction = segment.normalized()
        quaternion = get_rotation_to(self.CAPSULE_VECTOR, direction)
        capsule.rotation = quaternion
        return capsule

    def execute(self, context):

        print("radius %f" % self.radius)

        source_bm = bmesh.new()
        source_bm.from_mesh(bpy.context.active_object.data)
        world_oriention = bpy.context.active_object.matrix_world.copy()

        mball = bpy.data.metaballs.new("Volumetric_metacapsules")
        mball.resolution = self.resolution
        mball.threshold = self.threshold
        meta_obj = bpy.data.objects.new("Volumetric_metacapsules", mball)

        for edge in source_bm.edges:
            fromV = mathutils.Vector(edge.verts[0].co)
            toV = mathutils.Vector(edge.verts[1].co)
            self.new_capsule(mball.elements, fromV, toV, self.radius)

        bpy.context.scene.collection.objects.link(meta_obj)
        bpy.context.view_layer.objects.active = meta_obj
        meta_obj.select_set(True)

        meta_obj.matrix_world = world_oriention
        return {'FINISHED'}


def menu_func(self, context):
    self.layout.operator(ToxicBlend_MetaVolume.bl_idname, icon='META_BALL')


def register():
    bpy.utils.register_class(ToxicBlend_MetaVolume)
    bpy.types.VIEW3D_MT_metaball_add.append(menu_func)


def unregister():
    bpy.utils.unregister_class(ToxicBlend_MetaVolume)
    bpy.types.VIEW3D_MT_metaball_add.remove(menu_func)


if __name__ == "__main__":
    register()
