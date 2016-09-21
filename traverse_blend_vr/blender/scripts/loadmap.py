
from bge import logic
from os.path import isfile


import sys
sys.path.append("/usr/local/lib/python3.4/dist-packages/pyinotify-0.9.6-py3.4.egg") 
import pyinotify
import asyncore
import os

class Loadmap:
    """
        @brief When a lib is loaded this callback is called after the lib is loaded
        @param status: Variable which holds the outcome of the async libload call
    """
    @staticmethod
    def asyncLibLoad_cb(status):
    	print("Library (%s) loaded in %.2fms." % (status.libraryName, status.timeTaken,))
    	print(status.progress)
    
    """
        @brief Reload a blendfile, effectively removing and loading the lib again.
    """
    @staticmethod
    def reload_lib(file_path):
        Loadmap.remove_lib(file_path)
        Loadmap.load_lib(file_path)
        
    
    """
        @biref Provided with a valid lib identifier removes the lib from the current scene
        @param lib: libidentifier
        @return 
    """    
    @staticmethod
    def remove_lib(file_path):
        if Loadmap.isBlend(file_path):
            print("removing lib %s" % Loadmap.makeLibName(file_path))
            if(Loadmap.makeLibName(file_path) in logic.LibList()):
                logic.LibFree(Loadmap.makeLibName(file_path))        
            else:
                print("lib was not loaded")


    """
        @brief Loads a unique instance of a blend.
        
        @param filpath path to blend file
        @return name of loaded lib 
    """      
    @staticmethod
    def load_lib(file_path):

        if not (Loadmap.isFile(file_path) and Loadmap.isBlend(file_path)):
            print("no such file or is no blend file")
            print(file_path)
            return
        
        print("load lib %s" % Loadmap.makeLibName(file_path))   
        try:
            f = open(file_path, 'rb')
            fb = f.read()
            identifier = Loadmap.makeLibName(file_path)
            print("libload %s " % identifier)
            logic.LibLoad(identifier, 'Scene', fb, load_actions=False, verbose=True, load_scripts=False, async=False).onFinish = Loadmap.asyncLibLoad_cb
        finally:
            f.close() 
        
                
        # set the physics of the loadad map to no colliiosn                
    #    print(bpy.data.objects)
    #    for ob in bpy.context.scene.objects:
    #        if ob.type == 'MESH' and (ob.name.startswith("mapFrom")):
    #            ob.game.physics_type = 'NO_COLLISION'
    
        return identifier
        
    @staticmethod
    def makeLibName(path):
        if Loadmap.isBlend(path):
            return path[:-6]+str('_lib')
        else:
            return ""
        
    @staticmethod
    def isFile(path):
        if not isfile(path):
            print("no such file: " + path + " libloadfailed")
            return False
        return True
        
    @staticmethod
    def isBlend(path):
        if path[-6:] != '.blend':
            print("file does not end with .blend")
            return False
        return True
        



class EventHandler(pyinotify.ProcessEvent):
    
    def process_IN_CREATE(self, event):
        # load new lib
        print ("Creating:", event.pathname)
        Loadmap.load_lib(event.pathname)

    def process_IN_DELETE(self, event):
        # remove lib
        print ("Removing:", event.pathname)
        Loadmap.remove_lib(event.pathname)

    def process_IN_MODIFY(self, event):
        # reload map
        print ("Modified:", event.pathname)
        Loadmap.reload_lib(event.pathname)

    def process_IN_CLOSE_WRITE(self, event):
        # reload map
        print ("Close written:", event.pathname)

    def process_IN_MOVED_TO(self, event):
        # load pathname
        # delet src_pathname
        print ("Moved to: ", event.pathname,"from",event.src_pathname)
        Loadmap.remove_lib(event.src_pathname)
        Loadmap.load_lib(event.pathname)
        
    def process_IN_MOVED_FROM(self, event):
        print ("IN MOVED FROM")
        return
            

def initial_sweep(path):
    initial_sweep_count = 0
    for filename in os.listdir(path):
        if Loadmap.isBlend(filename):
            Loadmap.load_lib(path + "/" + filename)
            initial_sweep_count += 1
    print("the initial sweep loaded ",initial_sweep_count,"libraries of ",len(os.listdir(path)))

    

def setup(contr):
    owner = contr.owner
    
    print("initial sweep") # check if there are already blendfiles in the folder
    initial_sweep(owner['load_map.dirtowatch'])
    print("init pyinotify")
    owner['load_map.mask'] = pyinotify.IN_DELETE | pyinotify.IN_CREATE | pyinotify.IN_MODIFY | pyinotify.IN_MOVED_TO | pyinotify.IN_MOVED_FROM # watched events
    
    owner['load_map.wm'] = pyinotify.WatchManager()  # Watch Manager
    owner['load_map.notifier'] = pyinotify.AsyncNotifier(owner['load_map.wm'], EventHandler())
    owner['load_map.wm'].add_watch(owner['load_map.dirtowatch'], owner['load_map.mask'])
    

def main(contr):
    owner = contr.owner
    if 'load_map.wm' not in owner:
        setup(contr)
    else:
        asyncore.poll()
        

def loadall(contr):
    for i in range(100):
        if not loadmap(contr):
            print("no more maps to load")            
            break

    
if __name__ == "__main__":
    main(logic.getCurrentController())
