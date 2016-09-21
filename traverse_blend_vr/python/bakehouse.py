# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 18:46:52 2015

@author: eruff

"""

import bpy
from os import listdir
from os.path import isfile, join

import bakery

from time import time

from createBakery import createBakery

def bake(obj):
    

    bpy.context.scene.objects.active = obj
    
    # deselect all other objects in the scene
    for ob in bpy.data.objects:
        ob.select = False
    
    print('test the bakery')
    bake_config = {'filepath':'/tmp/textures/',
                   'baked_texture': 'baked_image'+str(time()),
                   'baked_texture_img': 'baked_image'+str(time())+'.png',
                   'texture':'baked_texture_'+str(time()),
                   'material':'baked_material_'+str(time()),
                   'uv_map':'baked_uv_map_'+str(time()),
                   'unwrap':'lightmap_pack'}
          
    # set up the scenery to bake a texture to an object
    print('create the bakery')
    cBakery = createBakery()
    # get the object which should be baked
#    obj = bpy.data.objects[objName]
    # bake the texture
    bakery.bake(obj,bake_config)
    # remove the scenery again
    print("clean up the bakery")
    del cBakery    

if __name__ == "__main__":
    
    fcstart = time()


    blendfiles = "/tmp/octomaptest/"
    
    blendfile_list = [f for f in listdir(blendfiles) if isfile(join(blendfiles,f))]   

#    blendfiles = "/tmp/"
#    blendfile_list = ['createMeshTest_fromData_lvl16.blend']

    for bf in blendfile_list:
        cstart = time()
        f = bpy.ops.wm.open_mainfile(filepath=blendfiles + bf,use_scripts=False)
        
        scene = bpy.context.scene
    
        obs = []
        for ob in scene.objects:
                # take the first mesh objects and bake it
                if ob.type == 'MESH':
                    bake(ob)
                    # save the new file again
                    bpy.ops.wm.save_mainfile(filepath=blendfiles + bf, check_existing=False)
                    break
        del f
        print("bakeing took",time()- cstart)
    print("the bakehouse took",time()- fcstart)