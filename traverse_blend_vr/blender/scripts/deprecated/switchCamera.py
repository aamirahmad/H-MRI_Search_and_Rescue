import bge
from bge import logic,events

if __name__ == "__main__":
    JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED
    keyboard = logic.keyboard

    if keyboard.events[events.CKEY] == JUST_ACTIVATED:

        scene = bge.logic.getCurrentScene()
        # set active camera    
        for i,cam in zip(range(len(scene.cameras)),scene.cameras):
            if cam == scene.active_camera:            
                camID = (i + 1) % len(scene.cameras)
                break
        
        camera = scene.objects[str(scene.cameras[camID])]
            
        try:
            scene.active_camera = camera            
        except:
            pass