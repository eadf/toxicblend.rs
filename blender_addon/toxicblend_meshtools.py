# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####
#
# The blender addon infrastructure of this file was copied from addons by Vladimir Spivak (cwolf3d)
# Germano Cavalcante (mano-wii), Florian Meyer (testscreenings), Brendon Murphy (meta-androcto) and Bart Crouch


# To install this addon you need to install 'protobuf' and 'grpcio' to the blender python installation.
# This is an example of how this can be done: Open the script tab in blender and run this:
# """
# import subprocess
#  import sys
#  import os
#  import site
#
#  print("python executable:", sys.executable)
#
#  # upgrade pip
#  subprocess.call([sys.executable, "-m", "ensurepip"])
#  subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
#
#  # install required packages
#  subprocess.call([sys.executable, "-m", "pip", "install", "protobuf"])
#  subprocess.call([sys.executable, "-m", "pip", "install", "grpcio"])
# """
#
# You will also have to copy the toxicblend_pb2.py and toxicblend_pb2_grpc.py files to the
# site packages of the same python environment.
# See install_as_blender_addon.md for examples


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

bl_info = {
    "name": "Toxicblend MeshTools",
    "author": "EADF",
    "version": (0, 0, 1),
    "blender": (2, 92, 0),
    "location": "View3D > Sidebar > Edit Tab / Edit Mode Context Menu",
    "warning": "",
    "description": "Tools for handling lines and linestrings in edit mesh mode",
    "doc_url": "{BLENDER_MANUAL_URL}/addons/mesh/toxicblend_meshtools.html",
    "category": "Mesh",
}

SOCKET = 'localhost:50069'


class ToxicblendException(Exception):
    def __init__(self, message):
        self.message = str(message)


# ########################################
# ##### General functions ################
# ########################################

# gather initial data
def initialise():
    active_object = bpy.context.active_object
    if 'MIRROR' in [mod.type for mod in active_object.modifiers if mod.show_viewport]:
        # ensure that selection is synced for the derived mesh
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(active_object.data)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    return active_object, bm


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


# make an edge key object
def make_key(v0, v1):
    if v0 < v1:
        return v0, v1
    else:
        return v1, v0


IDENTITY4x4 = mathutils.Matrix.Identity(4)


def build_pb_matrix(bpy_object, pb_model):
    """ Build a PB matrix from a blender object"""
    bm = bpy_object.matrix_world
    if bm != IDENTITY4x4:
        pbm = pb_model.worldOrientation
        pbm.m00, pbm.m01, pbm.m02, pbm.m03 = bm[0][0], bm[0][1], bm[0][2], bm[0][3]
        pbm.m10, pbm.m11, pbm.m12, pbm.m13 = bm[1][0], bm[1][1], bm[1][2], bm[1][3]
        pbm.m20, pbm.m21, pbm.m22, pbm.m23 = bm[2][0], bm[2][1], bm[2][2], bm[2][3]
        pbm.m30, pbm.m31, pbm.m32, pbm.m33 = bm[3][0], bm[3][1], bm[3][2], bm[3][3]


def build_pb_model(bpy_object, bm, pb_model):
    """ Build a PB model from a blender object"""
    pb_model.name = bpy_object.name
    build_pb_matrix(bpy_object, pb_model)
    for v in bm.verts:
        pb_vertex = pb_model.vertices.add()
        pb_vertex.x = v.co.x
        pb_vertex.y = v.co.y
        pb_vertex.z = v.co.z

    sent_edges = set()  # set of tuple: (vertex.index, vertex.index). Lowest vertex.index first
    for f in bm.faces:
        pb_face = pb_model.faces.add()
        prev_vertex = None
        for v in f.verts:
            pb_face.vertices.append(v.index)
            if prev_vertex:
                sent_edges.add(make_key(prev_vertex, v.index))

            prev_vertex = v.index
            if prev_vertex and len(f.verts) > 0:
                first_vertex = f.verts[0].index
                sent_edges.add(make_key(first_vertex, prev_vertex))

    for e in bm.edges:
        f = e.verts[0].index
        t = e.verts[1].index
        key = make_key(f, t)
        if key not in sent_edges:
            pb_face = pb_model.faces.add()
            pb_face.vertices.append(f)
            pb_face.vertices.append(t)


def get_pydata(pb_model, only_edges=False):
    """Convert the received proto buffer data into something useful"""
    rv_vertices = [(v.x, v.y, v.z) for v in pb_model.vertices]
    rv_edges = []
    rv_faces = []
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
                #print("Adding face:", vertices)
                rv_faces.append(vertices)

    mat = IDENTITY4x4.copy()
    if pb_model.HasField("worldOrientation"):
        pbm = pb_model.worldOrientation
        mat[0][0], mat[0][1], mat[0][2], mat[0][3] = pbm.m00, pbm.m01, pbm.m02, pbm.m03
        mat[1][0], mat[1][1], mat[1][2], mat[1][3] = pbm.m10, pbm.m11, pbm.m12, pbm.m13
        mat[2][0], mat[2][1], mat[2][2], mat[2][3] = pbm.m20, pbm.m21, pbm.m22, pbm.m23
        mat[3][0], mat[3][1], mat[3][2], mat[3][3] = pbm.m30, pbm.m31, pbm.m32, pbm.m33
    return rv_vertices, rv_edges, rv_faces, mat


def handle_response(pb_message):
    for option in pb_message.options:
        if option.key == "ERROR":
            raise ToxicblendException(str(option.value))


def handle_received_object(active_object, pb_message, remove_doubles_threshold=None, set_origin_to_cursor=False):
    only_edges = False
    for option in pb_message.options:
        if option.key == "ERROR":
            raise ToxicblendException(str(option.value))
        if option.key == "ONLY_EDGES" and option.value == "True":
            only_edges = True

    # active_object.select = False

    if len(pb_message.models) == 1:
        pb_model = pb_message.models[0]
        (vertices, edges, faces, matrix) = get_pydata(pb_model, only_edges)
        if len(faces) > 0 or len(edges) > 0:
            new_mesh = bpy.data.meshes.new(pb_model.name)
            # ob = bpy.data.objects.new(pb_model.name, new_mesh)
            old_mesh = active_object.data
            # active_object.active = ob
            # ob.select = True
            print("vertices:", len(vertices))
            print("edges:", len(edges))
            print("faces:", len(faces))
            new_mesh.from_pydata(vertices, edges, faces)
            new_mesh.update(calc_edges=True)
            bm = bmesh.new()
            bm.from_mesh(new_mesh)
            bpy.ops.object.mode_set(mode='OBJECT')
            bm.to_mesh(active_object.data)
            bpy.ops.object.mode_set(mode='EDIT')
            # = new_mesh
            # print("active_object.update_from_editmode():", active_object.update_from_editmode())
            if not (old_mesh.users or old_mesh.use_fake_user):
                bpy.data.meshes.remove(old_mesh)
                print("removed old mesh")
            else:
                print("did not remove old mesh")

            if matrix:
                active_object.matrix_world = matrix

            if remove_doubles_threshold and remove_doubles_threshold > 0:
                # sometimes 'mode_set' does not take right away  :/
                # bpy.ops.object.editmode_toggle()
                bpy.ops.object.mode_set(mode='EDIT')
                bpy.ops.mesh.remove_doubles(threshold=remove_doubles_threshold)
                bpy.ops.object.editmode_toggle()
                bpy.ops.object.mode_set(mode='OBJECT')
            if set_origin_to_cursor:
                bpy.ops.object.origin_set(type='ORIGIN_CURSOR')


# ########################################
# ##### Operators ########################
# ########################################
class ToxicBlend_debug_object(bpy.types.Operator):
    """Simple object debug"""
    bl_idname = "mesh.toxicblend_meshtools_debug_object"
    bl_label = "Debug object (check console for messages)"
    bl_description = "Checks a mesh for anomalies, double edges etc. Will print results to the console. (offline plugin)"
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        problem_found = False
        active_object = context.view_layer.objects.active
        bm = bmesh.new()
        bm.from_mesh(active_object.data)
        object_name = active_object.name
        print("ToxicBlend_debug_object:", object_name)
        print("vertices:")
        bm.verts.ensure_lookup_table()
        for voi in range(0, len(bm.verts)):
            v = bm.verts[voi]
            if v.index != voi:
                print(
                    "%d:(%f, %f, %f) %s %d" % (v.index, v.co.x, v.co.y, v.co.z, "index does not match position:", voi))
                problem_found = True

        if len(bm.faces) == 0:
            print("No faces")
        else:
            for f in bm.faces:
                indices = []
                for v in f.verts:
                    indices.append(str(v.index))

        if len(bm.edges) == 0:
            print("No edges")
        else:
            print("Edges:")
            edge_map = {}
            for e in bm.edges:
                indices = []
                for v in e.verts:
                    indices.append(str(v.index))
                if len(indices) != 2 or indices[0] == indices[1]:
                    print("%s %s" % (",".join(indices), "problematic!!"))
                    problem_found = True
                else:
                    lowI = min(int(indices[0]), int(indices[1]))
                    highI = max(int(indices[0]), int(indices[1]))
                    # print("%d to %d " % (lowI,highI))
                    key = "%d-%d" % (lowI, highI)
                    if key in edge_map:
                        print("%s %s" % (",".join(indices), "double edge!!"))
                        problem_found = True

                    edge_map[key] = True
        if problem_found:
            self.report({'WARNING'}, "Problems were detected, check console")
        else:
            self.report({'INFO'}, "No problems detected")
        return {'FINISHED'}


class ToxicBlend_SelectEndVertices(Operator):
    """Selects all vertices that are only connected to one other vertex (offline plugin)"""
    bl_idname = "mesh.toxicblend_meshtools_select_end_vertices"
    bl_label = "Select end vertices"
    bl_description = "Selects all vertices that are only connected to one other vertex (offline plugin)"
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):

        # Get the active mesh
        obj = bpy.context.edit_object
        me = obj.data

        # Get a BMesh representation
        bm = bmesh.from_edit_mesh(me)
        bpy.ops.mesh.select_all(action='DESELECT')

        if len(bm.edges) > 0 or len(bm.faces) > 0:
            vertex_connections = array.array('i', (0 for i in range(0, len(bm.verts))))
            for e in bm.edges:
                for vi in e.verts:
                    vertex_connections[vi.index] += 1
            for f in bm.faces:
                for vi in f.verts:
                    vertex_connections[vi.index] += 1

            for vi in range(0, len(vertex_connections)):
                if vertex_connections[vi] < 2:
                    bm.verts[vi].select = True

        # Show the updates in the viewport
        bmesh.update_edit_mesh(me, False)

        return {'FINISHED'}


class ToxicBlend_SelectIntersectionVertices(Operator):
    """Selects all vertices that are connected to more than two other vertices (offline plugin)"""
    bl_idname = "mesh.toxicblend_meshtools_select_intersection_vertices"
    bl_label = "Select intersection vertices"
    bl_description = "Selects all vertices that are connected to more than two other vertices (offline plugin)"
    bl_options = {'REGISTER', 'UNDO'}  # enable undo for the operator.

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):

        # Get the active mesh
        obj = bpy.context.edit_object
        me = obj.data

        # Get a BMesh representation
        bm = bmesh.from_edit_mesh(me)
        bpy.ops.mesh.select_all(action='DESELECT')

        if len(bm.edges) > 0 or len(bm.faces) > 0:
            vertex_connections = array.array('i', (0 for i in range(0, len(bm.verts))))
            for e in bm.edges:
                for vi in e.verts:
                    vertex_connections[vi.index] += 1
            for f in bm.faces:
                for vi in f.verts:
                    vertex_connections[vi.index] += 1

            for vi in range(0, len(vertex_connections)):
                if vertex_connections[vi] > 2:
                    bm.verts[vi].select = True

        # Show the updates in the viewport
        bmesh.update_edit_mesh(me, False)

        return {'FINISHED'}


# 2d_outline operator
class Toxicblend_2D_Outline(Operator):
    bl_idname = "mesh.toxicblend_meshtools_2d_outline"
    bl_label = "2D Outline"
    bl_description = "Outline 2d geometry, the geometry must be flat and on a plane intersecting origin"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='2d_outline')
                opt = command.options.add()
                opt.key = "a key"
                opt.value = "a value"
                build_pb_model(active_object, active_mesh, command.models.add())

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# Voronoi operator
class Toxicblend_Voronoi(Operator):
    bl_idname = "mesh.toxicblend_meshtools_voronoi"
    bl_label = "Voronoi"
    bl_description = "Generates a voronoi diagram from the input model. The operation takes one flat model as input (lines and points). \nOnly internal edges will be preserved so encompass the model with an outer perimeter."
    bl_options = {'REGISTER', 'UNDO'}

    remove_externals: BoolProperty(
        name="Remove external edges",
        description="Remove edges connected or indirectly connected to 'infinite' edges. Edges inside input geometry are always considered 'internal'",
        default=True
    )

    distance: FloatProperty(
        name="Discretization Distance",
        description="Discretization distance as a percentage of the larges axis, used in the discretization process of curved edges",
        default=0.001,
        min=0.000101,
        max=0.999999,
        precision=6,
        subtype='PERCENTAGE'
    )

    secondary_edges: BoolProperty(
        name="Remove secondary edges",
        description="Remove edges that are considered secondary, as they are generated by an input segment and its endpoint",
        default=False
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='voronoi')

                opt = command.options.add()
                opt.key = "VORONOI_DISCRETE_DISTANCE"
                opt.value = str(self.distance)

                opt = command.options.add()
                opt.key = "REMOVE_EXTERNALS"
                opt.value = str(self.remove_externals).lower()

                opt = command.options.add()
                opt.key = "REMOVE_SECONDARY_EDGES"
                opt.value = str(self.secondary_edges).lower()

                build_pb_model(active_object, active_mesh, command.models.add())
                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# Knife Intersect operator
class Toxicblend_Knife_Intersect(Operator):
    bl_idname = "mesh.toxicblend_meshtools_knife_intersect"
    bl_label = "Knife Intersect"
    bl_description = "Knife cuts self intersecting geometry, the geometry must be flat and on a plane intersecting origin"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='knife_intersect')
                build_pb_model(active_object, active_mesh, command.models.add())

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# Centerline operator
class Toxicblend_Centerline(Operator):
    bl_idname = "mesh.toxicblend_meshtools_centerline"
    bl_label = "Centerline"
    bl_description = "Calculate centerline, the geometry must be flat and on a plane intersecting origin"
    bl_options = {'REGISTER', 'UNDO'}

    angle: FloatProperty(
        name="Angle",
        description="Edge rejection angle, edges with edge-to-segment angles larger than this will be rejected",
        default=math.radians(50.0),
        min=math.radians(0.000001),
        max=math.radians(89.999999),
        precision=6,
        subtype='ANGLE',
    )

    weld: BoolProperty(
        name="Weld the centerline to outline",
        description="Centerline and outline will share vertices if they intersect",
        default=True
    )

    remove_internals: BoolProperty(
        name="Remove internal edges",
        description="Remove edges internal to islands in the geometry",
        default=True
    )

    distance: FloatProperty(
        name="Distance",
        description="Discrete distance as a percentage of the AABB",
        default=0.005,
        min=0.0001,
        max=4.9999,
        precision=6,
        subtype='PERCENTAGE'
    )

    simplify: BoolProperty(
        name="Simplify line strings",
        description="Simplify voronoi edges connected as in a line string. The 'distance' property is used.",
        default=True
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='centerline')
                build_pb_model(active_object, active_mesh, command.models.add())
                opt = command.options.add()
                opt.key = "ANGLE"
                opt.value = str(math.degrees(self.angle))
                opt = command.options.add()
                opt.key = "REMOVE_INTERNALS"
                opt.value = str(self.remove_internals).lower()
                opt = command.options.add()
                opt.key = "DISTANCE"
                opt.value = str(self.distance)
                opt = command.options.add()
                opt.key = "SIMPLIFY"
                opt.value = str(self.simplify).lower()
                opt = command.options.add()
                opt.key = "WELD"
                opt.value = str(self.weld).lower()

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# Centerline operator
class Toxicblend_Voronoi_Mesh(Operator):
    bl_idname = "mesh.toxicblend_meshtools_voronoi_mesh"
    bl_label = "Voronoi Mesh"
    bl_description = "Calculate voronoi diagram and add mesh, the geometry must be flat and on a plane intersecting origin."
    bl_options = {'REGISTER', 'UNDO'}

    remove_externals: BoolProperty(
        name="Remove external edges",
        description="Remove edges connected or indirectly connected to 'infinite' edges. Edges inside input geometry are always considered 'internal'",
        default=True
    )

    distance: FloatProperty(
        name="Distance",
        description="Discrete distance as a percentage of the AABB",
        default=0.005,
        min=0.0001,
        max=4.9999,
        precision=6,
        subtype='PERCENTAGE'
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='voronoi_mesh')
                build_pb_model(active_object, active_mesh, command.models.add())
                opt = command.options.add()
                opt.key = "REMOVE_EXTERNALS"
                opt.value = str(self.remove_externals).lower()
                opt = command.options.add()
                opt.key = "DISTANCE"
                opt.value = str(self.distance)

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

# Centerline operator
class Toxicblend_Voxel(Operator):
    bl_idname = "mesh.toxicblend_meshtools_voxel"
    bl_label = "Voxel"
    bl_description = "Calculate voxel tubes around edges"
    bl_options = {'REGISTER', 'UNDO'}

    angle: FloatProperty(
        name="Angle",
        description="Edge rejection angle, edges with edge-to-segment angles larger than this will be rejected",
        default=math.radians(50.0),
        min=math.radians(0.000001),
        max=math.radians(89.999999),
        precision=6,
        subtype='ANGLE',
    )

    remove_internals: BoolProperty(
        name="Remove internal edges",
        description="Remove edges internal to islands in the geometry",
        default=True
    )

    distance: FloatProperty(
        name="Distance",
        description="Discrete distance as a percentage of the AABB",
        default=0.005,
        min=0.0001,
        max=4.9999,
        precision=6,
        subtype='PERCENTAGE'
    )

    simplify: BoolProperty(
        name="Simplify line strings",
        description="Simplify voronoi edges connected as in a line string. The 'distance' property is used.",
        default=True
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='voxel')
                build_pb_model(active_object, active_mesh, command.models.add())
                opt = command.options.add()
                opt.key = "ANGLE"
                opt.value = str(math.degrees(self.angle))
                opt = command.options.add()
                opt.key = "REMOVE_INTERNALS"
                opt.value = str(self.remove_internals).lower()
                opt = command.options.add()
                opt.key = "DISTANCE"
                opt.value = str(self.distance)
                opt = command.options.add()
                opt.key = "SIMPLIFY"
                opt.value = str(self.simplify).lower()

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

# 2d_outline operator
class Toxicblend_Simplify(Operator):
    bl_idname = "mesh.toxicblend_meshtools_simplify"
    bl_label = "Simplify"
    bl_description = "Simplify poly-lines"
    bl_options = {'REGISTER', 'UNDO'}

    distance: FloatProperty(
        name="Distance",
        description="Simplification distance",
        default=0.01,
        min=0.00000001,
        max=4.99999999,
        precision=6,
        subtype='PERCENTAGE'
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == 'MESH' and context.mode == 'EDIT_MESH'

    def invoke(self, context, event):
        # load custom settings
        settings_load(self)
        return self.execute(context)

    def execute(self, context):
        # initialise
        active_object, active_mesh = initialise()
        settings_write(self)
        try:
            with grpc.insecure_channel(SOCKET) as channel:
                stub = toxicblend_pb2_grpc.ToxicBlendServiceStub(channel)
                command = toxicblend_pb2.Command(command='simplify')
                opt = command.options.add()
                opt.key = "DISTANCE"
                opt.value = str(self.distance / 100.0)
                build_pb_model(active_object, active_mesh, command.models.add())

                response = stub.execute(command)
                handle_response(response)
                if len(response.models) > 0:
                    print("client received options: ", len(response.options), " models:", len(response.models))
                    handle_received_object(active_object, response)

            # cleaning up
            terminate()

            return {'FINISHED'}
        except ToxicblendException as e:
            self.report({'ERROR'}, e.message)
            return {'CANCELLED'}
        except grpc._channel._InactiveRpcError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# ########################################
# ##### GUI and registration #############
# ########################################

# menu containing all tools
class VIEW3D_MT_edit_mesh_toxicblend_meshtools(Menu):
    bl_label = "Toxicblend meshtools"

    def draw(self, context):
        layout = self.layout
        layout.operator("mesh.toxicblend_meshtools_simplify")
        layout.operator("mesh.toxicblend_meshtools_2d_outline")
        layout.operator("mesh.toxicblend_meshtools_knife_intersect")
        layout.operator("mesh.toxicblend_meshtools_centerline")
        layout.operator("mesh.toxicblend_meshtools_voronoi_mesh")
        layout.operator("mesh.toxicblend_meshtools_voronoi")
        #layout.operator("mesh.toxicblend_meshtools_voxel")
        layout.operator("mesh.toxicblend_meshtools_select_end_vertices")
        layout.operator("mesh.toxicblend_meshtools_select_intersection_vertices")
        layout.operator("mesh.toxicblend_meshtools_debug_object")


# panel containing all tools
class VIEW3D_PT_tools_toxicblend_meshtools(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Edit'
    bl_context = "mesh_edit"
    bl_label = "Toxicblend meshtools"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        lt = context.window_manager.tb_meshtools
        # todo: fts


# property group containing all properties for the gui in the panel
class TB_MeshToolsProps(PropertyGroup):
    """
    Fake module like class
    bpy.context.window_manager.tb_meshtools
    """
    # general display properties
    simplify_distance: FloatProperty(
        name="Distance",
        description="Simplification distance",
        default=0.01,
        min=0.0,
        max=5.0,
        precision=6,
        subtype='PERCENTAGE'
    )

    centerline_angle: FloatProperty(
        name="Angle",
        description="Edge rejection angle, edges with edge-to-segment angles larger than this will be rejected",
        default=math.radians(50.0),
        min=math.radians(0.0),
        soft_min=math.radians(0.0),
        max=math.radians(90.0),
        soft_max=math.radians(90.0),
        precision=6,
        subtype='ANGLE',
    )

    centerline_remove_internals: BoolProperty(
        name="Remove internal edges",
        description="Remove edges internal to islands in the geometry",
        default=True
    )

    centerline_distance: FloatProperty(
        name="Distance",
        description="Discrete distance as a percentage of the AABB",
        default=0.01,
        min=0.005,
        max=99.0,
        precision=6,
        subtype='PERCENTAGE'
    )

    centerline_simplify: BoolProperty(
        name="Simplify line strings",
        description="Simplify voronoi edges connected as in a line string. The 'distance' property is used.",
        default=True
    )

    centerline_weld: BoolProperty(
        name="Weld the centerline to outline",
        description="Centerline and outline will share vertices if they intersect",
        default=True
    )

    voronoi_mesh_distance: FloatProperty(
        name="Distance",
        description="Discrete distance as a percentage of the AABB",
        default=0.005,
        min=0.0001,
        max=4.9999,
        precision=6,
        subtype='PERCENTAGE'
    )

    voronoi_mesh_remove_externals: BoolProperty(
        name="Remove external edges",
        description="Remove edges connected or indirectly connected to 'infinite' edges. Edges inside input geometry are always considered 'internal'",
        default=True
    )

    voronoi_remove_externals: BoolProperty(
        name="Remove external edges",
        description="Remove edges connected or indirectly connected to 'infinite' edges. Edges inside input geometry are always considered 'internal'",
        default=True
    )

    voronoi_distance: FloatProperty(
        name="Discretization Distance",
        description="Discretization distance as a percentage of the larges axis, used in the discretization process of curved edges",
        default=0.001,
        min=0.000101,
        max=0.999999,
        precision=6,
        subtype='PERCENTAGE'
    )

    voronoi_secondary_edges: BoolProperty(
        name="Remove secondary edges",
        description="Remove edges that are considered secondary, as they are generated by an input segment and its endpoint",
        default=False
    )


# draw function for integration in menus
def menu_func(self, context):
    self.layout.menu("VIEW3D_MT_edit_mesh_toxicblend_meshtools")
    self.layout.separator()


# define classes for registration
classes = (
    VIEW3D_MT_edit_mesh_toxicblend_meshtools,
    TB_MeshToolsProps,
    Toxicblend_Simplify,
    Toxicblend_2D_Outline,
    Toxicblend_Knife_Intersect,
    Toxicblend_Centerline,
    Toxicblend_Voronoi_Mesh,
    Toxicblend_Voronoi,
    #Toxicblend_Voxel,
    ToxicBlend_SelectEndVertices,
    ToxicBlend_SelectIntersectionVertices,
    ToxicBlend_debug_object,
)


# registering and menu integration
def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_edit_mesh_context_menu.prepend(menu_func)
    bpy.types.WindowManager.tb_meshtools = PointerProperty(type=TB_MeshToolsProps)


# unregistering and removing menus
def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    bpy.types.VIEW3D_MT_edit_mesh_context_menu.remove(menu_func)
    try:
        del bpy.types.WindowManager.tb_meshtools
    except Exception as e:
        print('unregister fail:\n', e)
        pass

# if __name__ == "__main__":
#    register()
