#!/usr/bin/python   
from __future__ import print_function
import bmesh
import bpy
import traceback
import mathutils
import math
import array
from bpy.types import (
    Operator,
    Menu,
    Panel,
    PropertyGroup,
)
from bpy.props import (
    BoolProperty,
    FloatProperty,
    PointerProperty,
)

# import gRPC
import grpc
import toxicblend.toxicblend_pb2_grpc as toxicblend_pb2_grpc
import toxicblend.toxicblend_pb2 as toxicblend_pb2
from bpy_extras.object_utils import AddObjectHelper

# To install this addon you need to install 'toxicblend', 'protobuf' and 'grpcio' to the blender python installation.
# This is an example of how this can be done: Open the script tab in blender and run this:
#
# """
# import subprocess
# import sys
# import os
# import site
#
# print("this is the path to the python executable: ", sys.executable)
#
# # upgrade pip
# subprocess.call([sys.executable, "-m", "ensurepip"])
# subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

# # install the pip package toxicblend + the dependencies protobuf and grpcio
# subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "toxicblend"])
# """
#
# Then you need to install this file as a blender addon as usual.
# See install_as_blender_addon.md details

bl_info = {
    'name': "Toxicblend - Add Lindenmayer systems",
    "author": "EADF",
    "version": (0, 0, 1),
    "blender": (2, 92, 0),
    # "location": "View3D > Sidebar > Edit Tab / Edit Mode Context Menu",
    "warning": "Communicates with a gRPC server on localhost",
    'description': 'Generates a parametric Lindenmayer systems graph.',
    # "doc_url": "{BLENDER_MANUAL_URL}/addons/mesh/toxicblend_lsystems.html",
    "category": "Mesh",
}
SERVER_URL = 'localhost:50069'


class ToxicblendException(Exception):
    def __init__(self, message):
        self.message = str(message)


# ########################################
# ##### General functions ################
# ########################################

# load custom tool settings
def settings_load(self):
    lt = bpy.context.window_manager.tb_meshtools
    tool = self.name.split()[0].lower()
    keys = self.as_keywords().keys()
    for key in keys:
        try:
            setattr(self, key, getattr(lt, tool + "_" + key))
        except AttributeError as e:
            print("tool:", tool, e)


# store custom tool settings
def settings_write(self):
    lt = bpy.context.window_manager.tb_meshtools
    tool = self.name.split()[0].lower()
    keys = self.as_keywords().keys()
    for key in keys:
        setattr(lt, tool + "_" + key, getattr(self, key))


# clean up and set settings back to original state
def terminate():
    # update editmesh cached data
    obj = bpy.context.active_object
    if obj.mode == 'EDIT':
        bmesh.update_edit_mesh(obj.data, loop_triangles=True, destructive=True)


IDENTITY4x4 = mathutils.Matrix.Identity(4)


def get_pydata(pb_model, only_edges=False, packed_faces=False):
    """Convert the received proto buffer data into something useful"""
    rv_vertices = [(v.x, v.y, v.z) for v in pb_model.vertices]
    rv_edges = []
    rv_faces = []
    if packed_faces:
        for face in pb_model.faces:
            for i in range(0, len(face.vertices), 3):
                rv_faces.append([face.vertices[i], face.vertices[i + 1], face.vertices[i + 2]])
    else:
        for f in pb_model.faces:
            vertices = []
            for v in f.vertices:
                if 0 <= v < len(rv_vertices):
                    vertices.append(v)
                else:
                    print("Vertex %d is unknown -> ignored" % (v,))
            no_vertices = len(vertices)
            if no_vertices > 1:
                if only_edges or no_vertices == 2:
                    if vertices[0] != vertices[1]:
                        rv_edges.append(vertices)
                    else:
                        print("FIXME: was asked to make an edge between two identical vertices: %s and %s" % (
                            vertices[0], vertices[1]))
                else:
                    # print("Adding face:", vertices)
                    rv_faces.append(vertices)

    mat = IDENTITY4x4.copy()
    if pb_model.HasField("worldOrientation"):
        pbm = pb_model.worldOrientation
        mat[0][0], mat[0][1], mat[0][2], mat[0][3] = pbm.m00, pbm.m01, pbm.m02, pbm.m03
        mat[1][0], mat[1][1], mat[1][2], mat[1][3] = pbm.m10, pbm.m11, pbm.m12, pbm.m13
        mat[2][0], mat[2][1], mat[2][2], mat[2][3] = pbm.m20, pbm.m21, pbm.m22, pbm.m23
        mat[3][0], mat[3][1], mat[3][2], mat[3][3] = pbm.m30, pbm.m31, pbm.m32, pbm.m33

    #print("rv_vertices:", rv_vertices)
    #print("rv_edges:", rv_edges)

    return rv_vertices, rv_edges, rv_faces, mat


def handle_response(pb_message):
    for option in pb_message.options:
        if option.key == "ERROR":
            raise ToxicblendException(str(option.value))


def handle_received_object(dest_mesh, pb_message, remove_doubles_threshold=None, set_origin_to_cursor=False):
    only_edges = False
    packed_faces = False
    for option in pb_message.options:
        if option.key == "ERROR":
            raise ToxicblendException(str(option.value))
        if option.key == "ONLY_EDGES" and option.value == "True":
            only_edges = True
        if option.key == "PACKED_FACES" and option.value == "True":
            packed_faces = True

    if len(pb_message.models) == 1:
        pb_model = pb_message.models[0]
        (vertices, edges, faces, matrix) = get_pydata(pb_model, only_edges, packed_faces)
        if len(faces) > 0 or len(edges) > 0:
            #new_mesh = bpy.data.meshes.new(pb_model.name)
            # ob = bpy.data.objects.new(pb_model.name, new_mesh)
            # active_object.active = ob
            # ob.select = True
            print("vertices:", len(vertices))
            print("edges:", len(edges))
            print("faces:", len(faces))
            dest_mesh.from_pydata(vertices, edges, faces)
            dest_mesh.update(calc_edges=True)
            bm = bmesh.new()
            bm.from_mesh(dest_mesh)
            #bpy.ops.object.mode_set(mode='OBJECT')
            bm.to_mesh(dest_mesh)
            #bpy.ops.object.mode_set(mode='EDIT')

            #bm = bmesh.new()
            #bm.from_mesh(new_mesh)
            #bpy.ops.object.mode_set(mode='OBJECT')
            #dest_mesh.to_mesh(dest_mesh)
            #bpy.ops.object.mode_set(mode='EDIT')


class TbAddLindenmayerSystems(bpy.types.Operator):
    """Adds Lindenmayer systems edges/curves from local toxicblend server"""
    bl_idname = "mesh.toxicblend_add_lindenmayer_systems"
    bl_label = "Toxicblend:Add Lindenmayer systems"
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    iterations = bpy.props.IntProperty(name="Iterations", default=4, min=1, max=15)

    location = bpy.props.FloatVectorProperty(
        name="Location",
        subtype='TRANSLATION',
    )

    rotation = bpy.props.FloatVectorProperty(
        name="Rotation",
        subtype='EULER',
    )

    # generic transform props
    align_items = (
        ('WORLD', "World", "Align the new object to the world"),
        ('VIEW', "View", "Align the new object to the view"),
        ('CURSOR', "3D Cursor", "Use the 3D cursor orientation for the new object")
    )

    align = bpy.props.EnumProperty(
        name="Align",
        items=align_items,
        default='WORLD',
        update=AddObjectHelper.align_update_callback,
    )

    cmd_variant_items = (("CANTOR_SETS", "cantor sets", "draw cantor sets"),
                         ("DRAGON_CURVE", "dragon curve", "draw dragon curve"),
                         ("FRACTAL_BINARY_TREE", "fractal binary tree", "draw fractal binary tree"),
                         ("FRACTAL_PLANT", "fractal plant", "draw fractal plant"),
                         ("KOCH_CURVE", "koch curve", "draw koch curve"),
                         ("RANDOM_FRACTAL_GENERATOR", "random fractal generator", "draw random fractal generator"),
                         ("SIERPINSKI_ARROWHEAD","sierpinski arrowhead", "draw sierpinski arrowhead"),
                         ("SIERPINSKI_TRIANGLE","sierpinski triangle", "draw sierpinski triangle")
                        )
    cmd_variant = bpy.props.EnumProperty(name="Variant", items=cmd_variant_items, default="DRAGON_CURVE")

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):

        settings_write(self)
        cursor_location = bpy.context.scene.cursor.location.copy()
        try:
            with grpc.insecure_channel(SERVER_URL) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='lsystems')
                opt = command.options.add()
                opt.key = "ITERATIONS"
                opt.value = str(self.iterations)
                opt = command.options.add()
                opt.key = "CURSOR_POS_X"
                opt.value = str(cursor_location.x)
                opt = command.options.add()
                opt.key = "CURSOR_POS_Y"
                opt.value = str(cursor_location.y)
                opt = command.options.add()
                opt.key = "CURSOR_POS_Z"
                opt.value = str(cursor_location.z)
                opt = command.options.add()
                opt.key = "CMD_VARIANT"
                opt.value = str(self.cmd_variant)

                response = stub.execute(command)
                handle_response(response)

                if len(response.models) > 0:
                    mesh = bpy.data.meshes.new("lsystems")
                    #bm = bmesh.new()
                    #bm.verts.ensure_lookup_table()

                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(mesh, response)

                    #bm.to_mesh(mesh)
                    mesh.update()

                    # add the mesh as an object into the scene with this utility module
                    from bpy_extras import object_utils
                    object_utils.object_data_add(context, mesh, operator=self)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


def menu_func(self, context):
    self.layout.operator(TbAddLindenmayerSystems.bl_idname, icon='MESH_DATA')


def register():
    bpy.utils.register_class(TbAddLindenmayerSystems)
    bpy.types.VIEW3D_MT_mesh_add.append(menu_func)


def unregister():
    bpy.utils.unregister_class(TbAddLindenmayerSystems)
    bpy.types.VIEW3D_MT_mesh_add.remove(menu_func)