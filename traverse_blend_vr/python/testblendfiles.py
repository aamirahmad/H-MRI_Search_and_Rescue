# -*- coding: utf-8 -*-
"""
Created on Fri Nov  6 14:08:18 2015

@author: eruff

"""

import time

import os
try:
    import  bpy
except ImportError as ImpErr:
    print(ImpErr)
    

if __name__ == "__main__":
    
    try:
        scn = bpy.context.scene
    except NameError as NameErr:
        print(NameErr)
        
    DIR = "/home/eruff/octoblend_tests/fr_campus/"
    DIR = "/tmp/octomaptest/"    
    testFile = '/tmp/octomaptest/octomapTest_25_fromData_lvl16.blend'
    #append object from .blend file

    name = os.listdir(DIR)[0]
    
    for name in [n for n in os.listdir(DIR) if os.path.isfile(os.path.join(DIR,n)) and n.endswith(".blend")]:
    
        filename = DIR+name
        
        try:
            with bpy.data.libraries.load(filename) as (data_from, data_to):
                data_to.objects = data_from.objects
            #link object to current scene
            for obj in data_to.objects:
        
                if obj is not None:
                    scn.objects.link(obj)   
                    scn.update()
        except NameError as err:
            print(err)