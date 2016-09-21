# -*- coding: utf-8 -*-
"""
Created on Wed Dec  2 19:58:30 2015

@author: eruff

"""
        
import bpy
from os import listdir
from os.path import isfile, join



blendfiles = "/tmp/octomaptest/"
section   = "/Object/"
obj_name    = "Object"

obj_file_path = "/home/eruff/workspace/morse/morse_viz/object_files"

blendfile_list = [f for f in listdir(blendfiles) if isfile(join(blendfiles,f))]   


    

for bf in blendfile_list:
    filepath  = blendfiles + bf + section + obj_name
    directory = blendfiles + bf + section
    filename  = obj_name
    

    for i in range(100):
        if i != 0:
            filename = obj_name + str("{0:.3f}".format(i/1000))[1:]
        else:
            filename = obj_name
        print(filepath,filename,directory)            
        try:
            bpy.ops.wm.append(filepath=filepath,filename=filename,directory=directory)
            
        except:
            print("there was a problem")
            pass

