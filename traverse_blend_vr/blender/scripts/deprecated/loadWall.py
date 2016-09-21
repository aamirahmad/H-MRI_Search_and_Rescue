from bge import logic,events
from numpy import random

def unique_libload(file_path):
    '''Loads a unique instance of a blend.
    INPUTS:
        - file_path - path to the blend file
    OUTPUTS:
        - added_objects, the object that were added
    '''
    scene = logic.getCurrentScene()
    old_objects = scene.objects + scene.objectsInactive
    
    f = open(file_path, 'rb').read()
    identifier = file_path.split('.')[0] + "_lib" + str(.0)

    while identifier in logic.LibList():
        id = identifier.split('.')
        identifier = id[0] + '.' + str(int(id[1])+1)
    logic.LibLoad(identifier, 'Scene', f)
    
    all_obj = scene.objects + scene.objectsInactive
    
    added_objects = []
    
    for o in all_obj:
        if o not in old_objects:
            added_objects.append(o)


    return added_objects
def load():
    logic.LibLoad("wall.blend","Scene")
    logic.LibNew('wall001','Mesh',['Monkey'])

def keyHits(key_code):
    status = logic.keyboard.events[key_code]
    return status == logic.KX_INPUT_JUST_ACTIVATED

def replace():
    scene = logic.getCurrentScene()
    cube = scene.objects["Cube.001"]
    scene.addObject("wall001", "Empty", 0)
    
    #cube.replaceMesh("Monkey")
    
def free(): 
    logic.LibFree("wall.blend") 
 
 
def moveObject(obj):
    pos = [random.randint(0,50),-1*random.randint(0,45),14]
    
    obj.localPosition = [0,0,8]
    obj.localOrientation = [0,0,0]
    
functions = [free,replace,load]

def main():
    
    # Just shortening names here
    keyboard = logic.keyboard
    JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED

    if keyboard.events[events.UKEY] == JUST_ACTIVATED:

        
        addedObjects = unique_libload('notEmpty.blend')
        #for a in addedObjects:
        #    moveObject(a)
        print("libList",logic.LibList())
        print("functions",functions)
        scene = logic.getCurrentScene()
        print("scene Objects",scene.objects)
        print("global dict",logic.globalDict)        
        print("object count", len(scene.objects))