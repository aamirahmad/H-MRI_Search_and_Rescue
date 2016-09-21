import bakery
import bpy
import imp

imp.reload(bakery)

config = {'filepath':'./textures/',
          'image_name': 'bakery_image',
          'baked_image': 'baked_image',
          'texture':'baked_texture',
          'material':'baked_material',
          'uv_map':'baked_uv_map',
          'unwrap':'lightmap_pack'}
          
obj = bpy.data.objects["Cube"]

        
bakery.bake(obj,config)