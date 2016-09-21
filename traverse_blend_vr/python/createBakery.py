# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 11:18:07 2015

@author: Eugen Ruff
"""



import bpy
from createObjects import createMeshFromOperator
import numpy as np
from time import time
import mathutils

def eulerToRad(euler):
    return euler/180*np.pi

class createBakery:
    
    def __init__(self):
        self.objects = []
        self.scn = bpy.context.scene
        self.construct()
        return
        
    def __del__(self):
        print('del')
        self.destruct()
        return
        
    def addLamp(self,loc,rot,lampName,lampType,lampEnergy,lampColor):
        # allow only certain lamp types
        if lampType not in ['POINT','SUN','SPOT','HEMI','AREA']:
            print ('NO SUCH TYPE')
            return
    
            
        # Create new lamp datablock
        lamp_data = bpy.data.lamps.new(name=lampName, type=lampType)
        
        lamp_data.energy = lampEnergy
        lamp_data.color = lampColor
        # Create new object with our lamp datablock
        lamp_object = bpy.data.objects.new(name=lampName, object_data=lamp_data)
        
        
        # Link lamp object to the scene so it'll appear in this scene
        self.scn.objects.link(lamp_object)
    
        
        # Place lamp to a specified location
        lamp_object.location = loc
        # Rotate the lamp according to the provided euler angles
        lamp_object.rotation_euler = rot

        
        
        # And finally select it make active
        lamp_object.select = True
        self.scn.objects.active = lamp_object
        
        return lamp_object
    
    def addPlane(self,name,origin,scale):
        verts = np.multiply([[1,1,0],[1,-1,0],[-1,-1,0],[-1,1,0]],scale)
        return createMeshFromOperator(name,origin,verts,[[0,1,2,3]])
    
    def construct(self):
        # create the light source(s)
        self.objects.append(self.addLamp((0,0,10.),(0.,0,0),'un_01','SUN',1.,(1.,1.,1.)))

#        self.objects.append(self.addLamp((-15.,0.,20.),(0.,eulerToRad(-40),0),'Hemi02','HEMI',0.5,(1.,1.,1.)))
        # create the floor plane 
        self.objects.append(self.addPlane('floor',(0,0,0),15))
        
    def destruct(self):
        # remove all bakery equipment 
        self.delObjects(self.objects)
        
    def delObjects(self,objs = []):
        # go into object mode
        bpy.ops.object.mode_set(mode='OBJECT')
        # deselect all objects
        bpy.ops.object.select_all(action='DESELECT')
    
        # select the objects we want to remove
        for obj in objs:
            obj.select = True
        # remove selected objects
        bpy.ops.object.delete() 
    
    
if __name__ == "__main__":
    print('create Object')
    #construct the bakery
    cBakery = createBakery()
    # desctruct
    del cBakery
    print(time(),'delete Object')
    

    
    

