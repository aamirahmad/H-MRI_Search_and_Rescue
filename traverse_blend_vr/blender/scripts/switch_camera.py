# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 11:03:23 2015

@author: eruff

"""
import bge
from bge import logic,events

def main(contr):
    JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED
    keyboard = logic.keyboard

    if keyboard.events[events.VKEY] == JUST_ACTIVATED:
        own = contr.owner
        if own['swc.deactivated']:
            print("sorry, switch camera is currently deactivated")
            return
    

    
        scene = bge.logic.getCurrentScene()
        # set active camera    
        for i,cam in zip(range(len(scene.cameras)),scene.cameras):
            if cam == scene.active_camera:            
                try:
                    scene.active_camera = scene.objects[str(scene.cameras[(i + 1) % len(scene.cameras)])]
                except:
                    pass
                break
    

if __name__ == "__main__":
    
    contr = logic.getCurrentController()
    main(contr)





                
        
