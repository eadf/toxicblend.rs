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
import toxicblend
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
    "version": (0, 0, 13),
    "blender": (2, 92, 0),
    # "location": "View3D > Sidebar > Edit Tab / Edit Mode Context Menu",
    "warning": "Communicates with a gRPC server on localhost",
    'description': 'Generates a parametric Lindenmayer systems graph.',
    # "doc_url": "{BLENDER_MANUAL_URL}/addons/mesh/toxicblend_lsystems.html",
    "category": "Mesh",
}
SERVER_URL = 'localhost:50069'

def check_toxicblend_version():
    if toxicblend.__version__ != '0.2.0':
        raise ToxicblendException("pip package toxicblend version 0.2.0 is required")

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
    if (not obj is None ) and obj.mode == 'EDIT':
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

    if len(pb_message.models) > 0:
         pb_model = pb_message.models[0]
    elif len(pb_message.models32) > 0:
        pb_model = pb_message.models32[0]
    else:
        raise ToxicblendException("No return models found")
    if not pb_model is None:
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

    iterations : bpy.props.IntProperty(name="Iterations", default=4, min=1, max=15)

    location : bpy.props.FloatVectorProperty(
        name="Location",
        subtype='TRANSLATION',
    )

    rotation : bpy.props.FloatVectorProperty(
        name="Rotation",
        subtype='EULER',
    )

    # generic transform props
    align_items = (
        ('WORLD', "World", "Align the new object to the world"),
        ('VIEW', "View", "Align the new object to the view"),
        ('CURSOR', "3D Cursor", "Use the 3D cursor orientation for the new object")
    )

    align : bpy.props.EnumProperty(
        name="Align",
        items=align_items,
        default='WORLD',
        update=AddObjectHelper.align_update_callback,
    )

    cmd_variant_items = (("DRAGON_CURVE", "Dragon curve", "draw dragon curve"),
                         ("DRAGON_CURVE_3D", "Dragon curve 3d", "draw a 3d dragon curve"),
                         ("LEVY_C_CURVE", "Lévy C curve", "draw Lévy C curve"),
                         ("LEVY_C_CURVE_3D", "Lévy C curve 3d", "draw Lévy C curve, just an experiment"),
                         ("FRACTAL_BINARY_TREE", "Fractal binary tree", "draw fractal binary tree"),
                         ("FRACTAL_BINARY_TREE_3D", "Fractal binary tree 3d", "draw a 3d fractal binary tree"),
                         ("FRACTAL_PLANT", "Fractal plant", "draw fractal plant"),
                         ("KOCH_CURVE", "Koch curve", "draw a Koch curve (keep iterations low)"),
                         ("KOCH_CURVE_3D", "Koch curve 3d", "draw a 3d Koch curve (keep iterations low)"),
                         ("KOCH_CURVE_ISLAND", "Koch curve island", "draw a quadratic Koch curve island"),
                         ("KOCH_CURVE_ISLAND_3D", "Koch curve island 3d", "draw a 3d variant of quadratic Koch curve island"),
                         ("HILBERT_CURVE_3D", "Hilbert curve 3d", "draw Hilbert curve 3d (keep iterations very low)"),
                         ("SIERPINSKI_GASKET","Sierpinski gasket", "draw Sierpinski gasket"),
                         ("SIERPINSKI_GASKET_3D","Sierpinski gasket 3d", "draw a 3f Sierpinski gasket"),
                         ("SIERPINSKI_TRIANGLE","Sierpinski triangle", "draw Sierpinski triangle"),
                         ("GOSPER_CURVE","Gosper curve", "draw Gosper curve (keep iterations low)"),
                         ("GOSPER_CURVE_3D","Gosper curve 3d", "draw a 3d Gosper curve (keep iterations low)"),
                         ("CUSTOM_TURTLE","Custom turtle", "draw using custom turtle")
                        )
    cmd_variant : bpy.props.EnumProperty(name="Variant", items=cmd_variant_items, default="DRAGON_CURVE")

    custom_turtle0 : bpy.props.StringProperty(name="custom turtle 0", default='token("X", Turtle::Nop)?')
    custom_turtle1 : bpy.props.StringProperty(name="custom turtle 1", default='token("Y", Turtle::Nop)?')
    custom_turtle2 : bpy.props.StringProperty(name="custom turtle 2", default='token("F", Turtle::Forward(1))?')
    custom_turtle3 : bpy.props.StringProperty(name="custom turtle 3", default='token("+", Turtle::Yaw(-90))?')
    custom_turtle4 : bpy.props.StringProperty(name="custom turtle 4", default='token("-", Turtle::Pitch(90))?')
    custom_turtle5 : bpy.props.StringProperty(name="custom turtle 5", default='axiom("F X")?')
    custom_turtle6 : bpy.props.StringProperty(name="custom turtle 6", default='rule("X","X + Y F +")?')
    custom_turtle7 : bpy.props.StringProperty(name="custom turtle 7", default='rule("Y","- F X - Y")?;')
    custom_turtle8 : bpy.props.StringProperty(name="custom turtle 8", default='round()')
    custom_turtle9 : bpy.props.StringProperty(name="custom turtle 9", default='')

    def invoke(self, context, event):
        # load custom settings
        #settings_load(self)
        return self.execute(context)

    def execute(self, context):

        check_toxicblend_version()
        #settings_write(self)
        cursor_location = bpy.context.scene.cursor.location.copy()
        channel_opt = [('grpc.max_send_message_length', 512 * 1024 * 1024),
                       ('grpc.max_receive_message_length', 512 * 1024 * 1024)]
        try:
            with grpc.insecure_channel(SERVER_URL, options=channel_opt) as channel:
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
                if self.cmd_variant == "CUSTOM_TURTLE":
                    opt = command.options.add()
                    opt.key = "CUSTOM_TURTLE"
                    opt.value = str(self.custom_turtle0) + "\n" +\
                                str(self.custom_turtle1) + "\n" +\
                                str(self.custom_turtle2) + "\n" +\
                                str(self.custom_turtle3) + "\n" +\
                                str(self.custom_turtle4) + "\n" +\
                                str(self.custom_turtle5) + "\n" +\
                                str(self.custom_turtle6) + "\n" +\
                                str(self.custom_turtle7) + "\n" +\
                                str(self.custom_turtle8) + "\n" +\
                                str(self.custom_turtle9)


                pb_response = stub.execute(command)
                handle_response(pb_response)

                if len(pb_response.models) == 0 and len(pb_response.models32) == 0:
                    raise ToxicblendException("No return models found")
                else:
                    mesh = bpy.data.meshes.new("lsystems")
                    #bm = bmesh.new()
                    #bm.verts.ensure_lookup_table()

                    print("client received options: ", len(pb_response.options), " models64:", len(pb_response.models), " models32:", len(pb_response.models32))
                    handle_received_object(mesh, pb_response)

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

class TbAddVoxelSdf(bpy.types.Operator):
    """Adds Lindenmayer systems edges/curves from local toxicblend server"""
    bl_idname = "mesh.toxicblend_add_voxel_sdf"
    bl_label = "Toxicblend:Add Voxel Sdf systems"
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    location : bpy.props.FloatVectorProperty(
        name="Location",
        subtype='TRANSLATION',
    )

    rotation : bpy.props.FloatVectorProperty(
        name="Rotation",
        subtype='EULER',
    )

    # generic transform props
    align_items = (
        ('WORLD', "World", "Align the new object to the world"),
        ('VIEW', "View", "Align the new object to the view"),
        ('CURSOR', "3D Cursor", "Use the 3D cursor orientation for the new object")
    )

    align : bpy.props.EnumProperty(
        name="Align",
        items=align_items,
        default='WORLD',
        update=AddObjectHelper.align_update_callback,
    )

    t_param: FloatProperty(
            name="t parameter",
            description="Thickness: abs(dot(sin(point), cos(point.zxy)) - b) - t",
            default=1.0,
            min=0.0001,
            max=4.9999,
            precision=6,
            subtype='FACTOR'
    )

    s_param: FloatProperty(
            name="s parameter",
            description="Scale",
            default=1.0,
            min=0.0001,
            max=4.9999,
            precision=6,
            subtype='FACTOR'
    )

    b_param: FloatProperty(
            name="b parameter",
            description="Tube thickness: abs(dot(sin(point), cos(point.zxy)) - b) - t",
            default=1.0,
            min=-5.000,
            max=5.000,
            precision=6,
            subtype='FACTOR'
    )
    x_param: FloatProperty(
             name="x parameter",
             description="cos(x) and sin(x) multiplier",
             default=1.0,
             min=-5.000,
             max=5.000,
             precision=6,
             subtype='FACTOR'
    )
    y_param: FloatProperty(
             name="y parameter",
             description="cos(y) and sin(y) multiplier",
             default=1.0,
             min=-5.000,
             max=5.000,
             precision=6,
             subtype='FACTOR'
     )
    z_param: FloatProperty(
             name="z parameter",
             description="cos(z) and sin(z) multiplier",
             default=1.0,
             min=-5.000,
             max=5.000,
             precision=6,
             subtype='FACTOR'
     )
    divisions: FloatProperty(
        name="Voxel Divisions",
        description="The longest axis of the model will be divided up into this number of voxels, the other axes will have proportionally number of voxels",
        default=100.0,
        min=50,
        max=400,
        precision=1,
        subtype='FACTOR'
    )
    #plug_ends: BoolProperty(
    #    name="Plug ends",
    #    description="Wall off outer edges with mesh",
    #    default=True
    #)

    def invoke(self, context, event):
        # load custom settings
        #settings_load(self)
        return self.execute(context)

    def execute(self, context):

        check_toxicblend_version()
        #settings_write(self)
        cursor_location = bpy.context.scene.cursor.location.copy()
        channel_opt = [('grpc.max_send_message_length', 512 * 1024 * 1024),
                       ('grpc.max_receive_message_length', 512 * 1024 * 1024)]
        try:
            with grpc.insecure_channel(SERVER_URL, options=channel_opt) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='sdf')

                opt = command.options.add()
                opt.key = "T"
                opt.value = str(self.t_param)

                opt = command.options.add()
                opt.key = "B"
                opt.value = str(self.b_param)

                opt = command.options.add()
                opt.key = "S"
                opt.value = str(self.s_param)

                opt = command.options.add()
                opt.key = "X"
                opt.value = str(self.x_param)

                opt = command.options.add()
                opt.key = "Y"
                opt.value = str(self.y_param)

                opt = command.options.add()
                opt.key = "Z"
                opt.value = str(self.z_param)

                opt = command.options.add()
                opt.key = "DIVISIONS"
                opt.value = str(self.divisions)

                #opt = command.options.add()
                #opt.key = "PLUG_ENDS"
                #opt.value = str(self.plug_ends)

                pb_response = stub.execute(command)
                handle_response(pb_response)

                if len(pb_response.models) == 0 and len(pb_response.models32) == 0:
                    raise ToxicblendException("No return models found")
                else:
                    mesh = bpy.data.meshes.new("sdf")
                    #bm = bmesh.new()
                    #bm.verts.ensure_lookup_table()

                    print("client received options: ", len(pb_response.options), " models64:", len(pb_response.models), " models32:", len(pb_response.models32))
                    handle_received_object(mesh, pb_response)

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
    self.layout.operator(TbAddVoxelSdf.bl_idname, icon='MESH_DATA')


def register():
    bpy.utils.register_class(TbAddLindenmayerSystems)
    bpy.utils.register_class(TbAddVoxelSdf)
    bpy.types.VIEW3D_MT_mesh_add.append(menu_func)


def unregister():
    bpy.utils.unregister_class(TbAddLindenmayerSystems)
    bpy.utils.unregister_class(TbAddVoxelSdf)
    bpy.types.VIEW3D_MT_mesh_add.remove(menu_func)
