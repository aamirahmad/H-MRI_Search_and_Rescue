#----------------------------------------------------------
# File objects.py
#----------------------------------------------------------
import bpy
import mathutils
from mathutils import Vector
from numpy import random

from octomap import  *
from time import *
global ocTree
global pathToMap

pathToMap = "/home/eruff/workspace/arhms_telecmt/src/telekyb/packages/telekyb_users/tk_eruff/tk_eruff/maps/"
maps = ['fr_078_tidyup.bt','fr_campus.ot']

from time import sleep
 
 
def createMeshFromPrimitive(name, origin,size):
    bpy.ops.mesh.primitive_cube_add(
        view_align=False, 
        enter_editmode=False, 
        location=origin, 
        rotation=(0, 0, 0))
 
    ob = bpy.context.object
    ob.name = name
    ob.scale = (size)
    me = ob.data
    me.name = name+'Mesh'
    return ob
 
 
def joinNDeselct():
    for ob in bpy.context.scene.objects:
        if ob.type == 'MESH' and (ob.name.startswith("Cube")):
            ob.select = True
            bpy.context.scene.objects.active = ob
        else:
            ob.select = False
    bpy.ops.object.join()
    bpy.ops.object.select_all(action='DESELECT')
    
def saveTo(filename):
    bpy.ops.wm.save_as_mainfile(filepath=filename)    
    
if __name__ == "__main__":
    z = 0.
    
    ocTree = OcTree(pathToMap+maps[0])
    
    print("tree resolution",ocTree.getResolution())
    print("number of nodes",ocTree.calcNumNodes())
    itr = ocTree.begin_tree()
    root_size = itr.getSize()
    maxDepth = 0
    count = 0
    start_time = time()
    print("start it",start_time)
    for i in itr:
        if i.getDepth() == 15:
            count += 1
            createMeshFromPrimitive('Cube', (i.getX(),i.getY(),i.getZ()),it.getSize())
        if maxDepth < i.getDepth():
            maxDepth = i.getDepth()
        
    print("stop it",time()-start_time)
    print("maxDepth",maxDepth,"count", count)    
    for i in range(1000):
        orig = (random.randint(-5,5),random.randint(-5,5),random.randint(-5,5))
        run(orig)
    
    joinNDeselct()
    saveTo("/home/eruff/workspace/morse/morse_viz/mapTest.blend")

    