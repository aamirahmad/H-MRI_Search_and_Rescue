# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 14:12:58 2015

@author: eruff

"""

#----------------------------------------------------------
# File objects.py
#----------------------------------------------------------
import bpy
import mathutils
from mathutils import Vector
 
def createMeshFromData(name, origin, verts, faces):
    # Create mesh and object
    me = bpy.data.meshes.new(name+'Mesh')
    ob = bpy.data.objects.new(name, me)
    ob.location = origin
    ob.show_name = True
 
    # Link object to scene and make active
    scn = bpy.context.scene
    scn.objects.link(ob)
    scn.objects.active = ob
    ob.select = True
 
    # Create mesh from given verts, faces.
    me.from_pydata(verts, [], faces)
    # Update mesh with new data
    me.update()    
    return ob
 
def createMeshFromOperator(name, origin, verts, faces):
    bpy.ops.object.add(
        type='MESH', 
        enter_editmode=False,
        location=origin)
    ob = bpy.context.object
    ob.name = name
    ob.show_name = True
    me = ob.data
    me.name = name+'Mesh'
 
    # Create mesh from given verts, faces.
    me.from_pydata(verts, [], faces)
    # Update mesh with new data
    me.update()    
    # Set object mode
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob
 
def createCubeFromPrimitive(name, origin):
    bpy.ops.mesh.primitive_cube_add(
        view_align=False, 
        enter_editmode=False, 
        location=origin, 
        rotation=(0, 0, 0))
 
    ob = bpy.context.object
    ob.name = name
    ob.show_name = False
    me = ob.data
    me.name = name+'Mesh'
    return ob
 
def createArmatureFromData(name, origin):
    # Create armature and object
    amt = bpy.data.armatures.new(name+'Amt')
    ob = bpy.data.objects.new(name, amt)
    ob.location = origin
    ob.show_name = True
 
    # Link object to scene and make active
    scn = bpy.context.scene
    scn.objects.link(ob)
    scn.objects.active = ob
    ob.select = True
 
    # Create single bone
    bpy.ops.object.mode_set(mode='EDIT')
    bone = amt.edit_bones.new('Bone')
    bone.head = (0,0,0)
    bone.tail = (0,0,1)
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob
 
def createArmatureFromOperator(name, origin):
    bpy.ops.object.add(
        type='ARMATURE', 
        enter_editmode=True,
        location=origin)
    ob = bpy.context.object
    ob.name = name
    ob.show_name = True
    amt = ob.data
    amt.name = name+'Amt'
 
    # Create single bone
    bone = amt.edit_bones.new('Bone')
    bone.head = (0,0,0)
    bone.tail = (0,0,1)
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob
 
def createArmatureFromPrimitive(name, origin):
    bpy.ops.object.armature_add()
    bpy.ops.transform.translate(value=origin)
    ob = bpy.context.object
    ob.name = name
    ob.show_name = True
    amt = ob.data
    amt.name = name+'Amt'
    return ob
 
def run(origo):
    origin = Vector(origo)
    (x,y,z) = (0.707107, 0.258819, 0.965926)
    verts = ((x,x,-1), (x,-x,-1), (-x,-x,-1), (-x,x,-1), (0,0,1))
    faces = ((1,0,4), (4,2,1), (4,3,2), (4,0,3), (0,1,2,3))
 
    cone1 = createMeshFromData('DataCone', origin, verts, faces)
    cone2 = createMeshFromOperator('OpsCone', origin+Vector((0,2,0)), verts, faces)
    cone3 = createMeshFromPrimitive('PrimCone', origin+Vector((0,4,0)))
 
    rig1 = createArmatureFromData('DataRig', origin+Vector((0,6,0)))
    rig2 = createArmatureFromOperator('OpsRig', origin+Vector((0,8,0)))
    rig3 = createArmatureFromPrimitive('PrimRig', origin+Vector((0,10,0)))
    return
 
if __name__ == "__main__":
    run((0,0,0))