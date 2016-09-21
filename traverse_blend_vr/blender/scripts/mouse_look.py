#Blender Game Engine 2.55 Simple Camera Look
#Created by Mike Pan: mikepan.com

# Use mouse to look around
# W,A,S,D key to walk around
# E and C key to ascend and decend



from bge import events,logic,render
from mathutils import *

import sys

def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    # get the object this script is attached to
    camera = contr.owner
    print(camera)
    scene = logic.getCurrentScene()
    if not scene:
        # not ready, main reload(blenderapi)
        return

    # Do not move the camera if the current view is using another camera
    print ("test")
    
    if str(camera) == str(scene.active_camera):
        return
    print ("test")
#    # Do not move the camera if another object has set move_cameraFP
#    for obj in keyboard_ctrl_objects:
#        if not obj['move_cameraFP']:
#            return

    # set camera position increment from the movement speed
    pos_inc = 0.15 #camera['Speed'] / 1 #blenderapi.getfrequency()

    # Get Blender keyboard sensor
#    keyboard = contr.sensors['All_Keys']

    keyboard = logic.keyboard
    ACTIVE = logic.KX_INPUT_ACTIVE
    # Default movement
    move_translation = [0.0, 0.0, 0.0]

    # Also add the corresponding key for an AZERTY keyboard
    if ACTIVE == keyboard.events[events.WKEY] or ACTIVE == keyboard.events[events.ZKEY]:
        move_translation[2] = -pos_inc
    elif ACTIVE == keyboard.events[events.SKEY]:
        move_translation[2] = pos_inc
    # Also add the corresponding key for an AZERTY keyboard
    elif ACTIVE == keyboard.events[events.AKEY] or ACTIVE ==  keyboard.events[events.QKEY]:
        move_translation[0] = -pos_inc
    elif ACTIVE == keyboard.events[events.DKEY]:
        move_translation[0] = pos_inc
    elif ACTIVE == keyboard.events[events.RKEY]:
        move_translation[1] = pos_inc
    elif ACTIVE == keyboard.events[events.FKEY]:
        move_translation[1] = -pos_inc
    else:
        move_translation[0] = 0
        move_translation[1] = 0
        move_translation[2] = 0

    print(move_translation)
    # The second parameter of 'applyMovement' determines
    #  a movement with respect to the object's local
    #  coordinate system
    
    camera.applyMovement( move_translation, True )

#        elif key[1] == blenderapi.input_just_activated():
#            # Other actions activated with the keyboard
#            # Reset camera to center
#            if ACTIVE == blenderapi.F8KEY and keyboard.positive:
#                reset_position(camera)
#            if ACTIVE == blenderapi.F7KEY and keyboard.positive:
#                look_robot(camera)


def rotate(contr):
    """ Read the movements of the mouse and apply them
        as a rotation to the camera. """
    # get the object this script is attached to
    camera = contr.owner

    scene = logic.getCurrentScene()
    if not scene:
        # not ready, main reload(blenderapi)
        return

    # Do not move the camera if the current view is using another camera
    if camera != scene.active_camera:
        return
    camera = scene.cameras["BodyCam"]
    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']
    # Get Blender keyboard sensor

    # Show the cursor
    mouse_visible = True

    # Hide the cursor while we control the camera
    mouse_visible = False
    if mouse.positive:
        # get width and height of game window
        width = render.getWindowWidth()
        height = render.getWindowHeight()

        # get mouse movement from function
        move = mouse_move(camera, mouse, width, height)

        # set mouse sensitivity
        sensitivity = 0.003 # camera['Sensitivity']
        print (camera.localOrientation)
        # Amount, direction and sensitivity
        leftRight = move[0] * sensitivity
        upDown = move[1] * sensitivity
        
        print(upDown,leftRight)
        # set the values
        camera.localOrientation = Euler((upDown, 0.0, leftRight),'XYZ')
        print(Euler((upDown, 0.0, leftRight),'XYZ'))

        # Center mouse in game window
        # Using the '//' operator (floor division) to produce an integer result
        
        render.setMousePosition(width//2, height//2)
        
    # Show the cursor
    mouse_visible = True

    # Set the cursor visibility
    render.showMouse(mouse_visible)
    


def mouse_move(camera, mouse, width, height):
    """ Get the movement of the mouse as an X, Y coordinate. """
    # distance moved from screen center
    # Using the '//' operator (floor division) to produce an integer result
    x = width//2 - mouse.position[0]
    y = height//2 - mouse.position[1]
    
    
    
    # intialize mouse so it doesn't jerk first time
    try:
        camera['mouseInit']
    except KeyError:
        x = 0
        y = 0
        # bug in Add Property
        # can't use True.  Have to use 1
        camera['mouseInit'] = 1

    #logger.debug("Read displacement: %s, %s" % (x, y))
    
    # return mouse movement
    return x, y


    