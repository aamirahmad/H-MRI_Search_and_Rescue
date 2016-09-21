# -*- coding: utf-8 -*-
"""
Created on Wed Nov 25 18:25:44 2015

@author: eruff

"""

# This script uses bmesh operators to make 2 links of a chain.

import bpy
import bmesh
import numpy as np
from operator import itemgetter 

from time import time
global face0
face0 = np.array([[0,1,3,2],[0,4,5,1],[0,2,6,4],[4,6,7,5],[1,3,7,5],[2,6,7,3]],dtype=np.int_)



def createBMesh(meshname,origin,verts):
    global face0
    # Make a new BMesh
    bm = bmesh.new()
    for i,v in enumerate(verts):

        bm.verts.new(v)
        if i%8 == 7:
            for f in face0:
                bm.faces.new(itemgetter(*f)(bm.verts[-8:]))            
#                 bmesh.ops.contextual_create(bm, geom=itemgetter(*f)(bm.verts[-8:]))
    # Done with creating the mesh, simply link it into the scene so we can see it
    
    # now find and merge nodes which are too close together    
    bmesh.ops.remove_doubles(bm,verts=bm.verts[:],dist=0.001)
    # fix the face problem

    
    # Finish up, write the bmesh into a new mesh
    me = bpy.data.meshes.new("Mesh")
    bm.to_mesh(me)
    bm.free()
    
    
    # Add the mesh to the scene
    scene = bpy.context.scene
    obj = bpy.data.objects.new("Object", me)
    scene.objects.link(obj)
    
    # Select and make active
    scene.objects.active = obj
    obj.select = True

    return




if __name__ == "__main__":
    # test data
    verts = []


    print (len(verts))
    cstart = time()
    createBMesh("name",(0,0,0),verts)
    print("took ",time() -cstart)






