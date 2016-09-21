# -*- coding: utf-8 -*-
"""
Created on Thu Dec  3 12:17:26 2015

@author: eruff

"""

import bpy
import os
from time import time


class Bakery:
    def __init__(self,obj,config):
        print("init bakery")
        self.obj = obj
        self.config = config
        return
        
    def createUVtextures(self,name):
        # add a new UV texture
        bpy.ops.mesh.uv_texture_add()
        # rename the uv texture
        bpy.context.object.data.uv_textures[-1].name = name
        
    def switchToEditMode(self):
        bpy.ops.object.mode_set(mode='EDIT')

    def switchToObjectMode(self):
        bpy.ops.object.mode_set(mode='OBJECT')
    
    def select_all(self,action):
        bpy.ops.mesh.select_all(action=action)

    def uvUnwrap(self,mode):
        self.switchToEditMode()
        self.select_all('SELECT')
        if(str(mode) == 'smart_project'):
            bpy.ops.uv.smart_project(angle_limit=66.0, island_margin=0.0, user_area_weight=0.0)
        elif(str(mode) == 'cube_project'):
            bpy.ops.uv.cube_project(cube_size=1.0, correct_aspect=True, clip_to_bounds=False, scale_to_bounds=False)
        elif(str(mode) == 'lightmap_pack'):
            bpy.ops.uv.lightmap_pack(PREF_CONTEXT='SEL_FACES', PREF_PACK_IN_ONE=True, PREF_NEW_UVLAYER=False, PREF_APPLY_IMAGE=False, PREF_IMG_PX_SIZE=512, PREF_BOX_DIV=12, PREF_MARGIN_DIV=0.1)
        else:
            print("No valid mode")
            return False

        return True

    def newImage(self,ImgName):
        return bpy.ops.image.new(name=self.config['baked_texture'], width=1024, height=1024, color=(1.0, 1.0, 1.0, 1.0), alpha=False, float=False)
        
    
    def bake(self,obj,imgName):
        me = obj.data
        me.uv_textures.active = me.uv_textures[self.config['uv_map']]
        
#        bpy.data.screens['UV Editing'].areas[1].spaces[0].image = bpy.data.images[self.config['baked_texture']] 

#        self.switchToObjectMode()
#        bpy.context.scene.objects.active = self.obj        

        self.switchToEditMode()
        bpy.ops.object.bake_image()
        
        

        
    def saveImage(self,image,relpath=""):
        abs_path = os.path.abspath(relpath)
        print('saving the image to ',abs_path)
        bpy.data.images[image].save_render(filepath=abs_path)
                                    
    


    def makeMatandTexture(self,matName,texName,relpath,uv_map):
        # Load image file.
        

        abs_path = os.path.abspath(relpath)
        try:
            print('loading texture from',abs_path)
            img = bpy.data.images.load(abs_path)
        except:
            raise NameError("Cannot load image %s" % abs_path)
     
        # Create image texture from image
        cTex = bpy.data.textures.new(texName, type = 'IMAGE')
        cTex.image = img
#        cTex.image = bpy.data.screens['UV Editing'].areas[1].spaces[0].image = bpy.data.images[self.config['baked_texture']]
     
        
        # Create material
        mat = bpy.data.materials.new(matName)

        # make the new material shadeless
        mat.use_shadeless = True

     
        # Add texture slot for color texture
        mtex = mat.texture_slots.add()
        mtex.texture = cTex
        mtex.texture_coords = 'UV'
        mtex.uv_layer = uv_map
        mtex.use_map_color_diffuse = True 
        mtex.use_map_color_emission = True 
        mtex.emission_color_factor = 0.5
        mtex.use_map_density = True 
        mtex.mapping = 'FLAT' 
        
        me = self.obj.data
        me.materials.append(mat)
     
        return    


    def cleanup(self):
        # remove all the materials we did not create
        # this makes the assumption that we created at least one material
        
        self.switchToObjectMode()
        for i in range(len(self.obj.material_slots)-1):
            bpy.context.scene.objects.active = self.obj
            self.obj.active_material_index = i
            bpy.ops.object.material_slot_remove()   
            
        

        # remove the unused texture image
        image = bpy.data.images[self.config['baked_texture']]
        
        if image.users:
            image.user_clear()
                
        bpy.data.images.remove(image)
        self.switchToEditMode()
        self.select_all('SELECT')
        bpy.data.screens['UV Editing'].areas[1].spaces[0].image = bpy.data.images[self.config['baked_texture_img']]                 

# used as static wrapper for the bakery
# baking the material of the object with a given setup to a texture
# and applying this texture as new material (whilst remove the old one)



def bake(obj,config):

    tstart = time()
    # make the object the active on
    bpy.context.scene.objects.active = obj
    
    # deselect all other objects in the scene
    for ob in bpy.data.objects:
        ob.select = False

    obj.select = True    
    bakery = Bakery(obj,config)
     
    # create a new UV texture to map the baking on
    bakery.createUVtextures(config['uv_map'])
    # switch to edit mode so we can start to unwrap the object
    bakery.switchToEditMode()    
    # select all of the object
    bakery.select_all('SELECT')

    # unwrap the object to the UV texture
    print("unwrapping the object .. using ", config['unwrap'])
    if not bakery.uvUnwrap(config['unwrap']):
        return
        
    
    # make a new image we project our object onto
    bakery.newImage(config['baked_texture'])
    bpy.data.screens['UV Editing'].areas[1].spaces[0].image = bpy.data.images[config['baked_texture']] 
    
    # now we can finally bake the image onto the UV map
    print("baking ... ")
    bakery.bake(obj,config['baked_texture'])
    # save the newly made image
    bakery.saveImage(config['baked_texture'],config['filepath']+config['baked_texture_img'])
    # now we make a new material to show the new baked uv map
    print("making new material and texture")
#    matName,texName,rel_path,uv_map
    bakery.makeMatandTexture(config['material'],config['texture'],config['filepath']+config['baked_texture_img'],config['uv_map'])
    # remove all materials but the last one
    bakery.cleanup()

    print("... done this in " ,time()-tstart,"s")
 


    
          
if __name__ == "__main__"    :
    print("test the bakery")
        
    bake_config = {'filepath':'/tmp/textures/',
                   'baked_texture': 'baked_image'+str(time()),
                   'baked_texture_img': 'baked_image'+str(time())+'.png',
                   'texture':'baked_texture_'+str(time()),
                   'material':'baked_material_'+str(time()),
                   'uv_map':'baked_uv_map_'+str(time()),
                   'unwrap':'lightmap_pack'}
          
    obj = bpy.data.objects["mapFromData_0"]
    bpy.context.scene.objects.active = obj

    bake(obj,bake_config)
     
#    bpy.context.scene.objects.active = obj