# -*- coding: utf-8 -*-
"""
Created on Thu Dec 10 13:28:57 2015

@author: eruff

"""

import bpy
from bge import logic
from time import time


# module entry point main function
def main(contr):

    # get the object this script is attached to
    owner = contr.owner
    # get a list of the children game objects

    scene = logic.getCurrentScene()
    THRESHOLD = owner['distance_threshold']
    cam = scene.active_camera # get the active camera
 
    obs = 0
    for ob in scene.objects: # check the distance fo all objects
        if str(ob.name) != 'Ground': # check if this object is a mesh
            obs += 1
   
            if cam.getDistanceTo(ob) > THRESHOLD: # check the distance from object to camera
                ob.visible = False # if the object is too far make it invisible
            elif not ob.visible: # if the object is invisible and close enough
                ob.visible = True # make it visible again
    

	
if __name__ == "__main__":
    # Non-Module Execution entry point (Script)
    cstart = time()
    
    if logic.getCurrentController().mode == 0:
        main(logic.getCurrentController())
    
 