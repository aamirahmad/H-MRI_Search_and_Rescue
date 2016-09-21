from bge import logic,events

SPEED = 0.15

if __name__ == "__main__":
    # The all keys thing without a keyboard sensor (but you will
    # need an always sensor with pulse mode on)
    active_camera = logic.getCurrentScene().active_camera
    
    owner = logic.getCurrentController().owner    
    # move only the actve cam
    if str(owner.name) == str(active_camera):
        localPos = [0,0,0]
        
        # Just shortening names here
        keyboard = logic.keyboard
        ACTIVE = logic.KX_INPUT_ACTIVE
    
        if keyboard.events[events.WKEY] == ACTIVE:
            localPos[2] -= SPEED
        elif keyboard.events[events.SKEY] == ACTIVE:
            localPos[2] += SPEED
        elif keyboard.events[events.AKEY] == ACTIVE:
            localPos[0] -= SPEED
        elif keyboard.events[events.DKEY] == ACTIVE:
            localPos[0] += SPEED            
        else:
            localPos = [0,0,0] # set the local position of the camera to zero
        
        owner.applyMovement(localPos,True)    