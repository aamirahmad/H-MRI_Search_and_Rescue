# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 12:46:19 2015

@author: Eugen Ruff
"""

import bpy

import bakery
from createBakery import createBakery

def main(objName):
    obj = bpy.data.objects[objName]

    bpy.context.scene.objects.active = obj
    
    # deselect all other objects in the scene
    for ob in bpy.data.objects:
        print (obj.name)
        ob.select = False
    
    print('test the bakery')
    bake_config = {'filepath':'/tmp/textures/',
                   'baked_texture': 'baked_image',
                   'baked_texture_img': 'baked_image.png',
                   'texture':'baked_texture',
                   'material':'baked_material',
                   'uv_map':'baked_uv_map',
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
    main()