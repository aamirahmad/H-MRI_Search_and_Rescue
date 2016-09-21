# -*- coding: utf-8 -*-
"""
Created on Fri Jul 24 16:48:47 2015

@author: eruff

"""

        
import bpy
from os import listdir
from os.path import isfile, join
from time import time,strftime ,gmtime

obj_file_path = "/home/eruff/workspace/morse/morse_viz/object_files"

obj_files = [ f[:-4] for f in listdir(obj_file_path) if isfile(join(obj_file_path,f)) and join(obj_file_path,f)[-4:] == ".obj"]   


ob_names = [ob.name for ob in bpy.data.objects]
start_time = time()
for obj in obj_files:
    if obj not in ob_names:
        bpy.ops.import_scene.obj(filepath=bpy.path.abspath('//')+"/object_files/"+str(obj)+".obj") 


print("loadObjFiles took ",time()-start_time)
